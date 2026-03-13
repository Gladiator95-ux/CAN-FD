/**
 * SAMC21 SLCAN Bridge - ASCII + DMA TX  (v3)
 * ===========================================
 * Base    : samc21_slcan_fd_v2.c  (proven working)
 * Change  : UART TX path replaced with DMAC channel 0
 *           Everything else is byte-for-byte identical to v2.
 *
 * WHY DMA FOR TX ONLY
 * -------------------
 * The problem in v2 was:
 *   uart_tx_poll() drains 32 bytes per call from the ring to SERCOM DATA.
 *   While uart_tx_poll() spins on INTFLAG_DRE, it blocks can_process_rx().
 *   Under heavy PCAN load, the ring fills faster than it drains ? frames
 *   pile up ? eventually dropped even with atomic commit.
 *
 * With DMA TX:
 *   Each SLCAN frame (up to 140 chars) is copied into tx_pool[slot].
 *   DMAC feeds bytes to SERCOM DATA autonomously, triggered by DRE.
 *   CPU never spins on DRE. can_process_rx() runs uninterrupted.
 *   uart_tx_poll() and uart_tx_dre_isr() are removed entirely.
 *
 * DMA TX MECHANISM (step by step)
 * ---------------------------------
 *  1. can_process_rx() builds SLCAN frame into FrameBuf (stack, 140 bytes).
 *  2. dma_tx_submit(buf, len) copies frame into tx_pool[head slot].
 *  3. If DMAC idle: configure descriptor, call dma_start_transfer_job().
 *     If DMAC busy: frame sits in slot queue, picked up by completion CB.
 *  4. DMAC waits for SERCOM DRE trigger (DATA register empty).
 *  5. DRE fires ? DMAC reads tx_pool[slot][i] ? writes to SERCOM DATA.
 *  6. SERCOM shifts byte out onto wire. DRE fires again for next byte.
 *  7. Repeat until all 'len' bytes sent.
 *  8. DMAC fires DMA_CALLBACK_TRANSFER_DONE (one interrupt per frame).
 *  9. dma_tx_complete_cb: advance tail, start next queued slot if any.
 *
 * CPU is involved at steps 2, 3, 9 only (~30 cycles total per frame).
 * Steps 4-8 run entirely in hardware, parallel with can_process_rx().
 *
 * NO COLLISION GUARANTEE
 * -----------------------
 * tx_pool has TX_POOL_DEPTH slots. CPU writes to head slot, DMAC reads
 * from tail slot. They are always different slots (enforced by the queue
 * count check). tx_q_head written by main loop only. tx_q_tail written
 * by DMA completion ISR only. tx_q_count guarded by disable/enable IRQ.
 *
 * RX PATH (UNCHANGED FROM v2)
 * ----------------------------
 * poll_uart_rx() reads SERCOM INTFLAG_RXC in the main loop, same as v2.
 * No DMA on RX — PC sends only short SLCAN commands, byte rate is tiny.
 */

#include <asf.h>
#include <string.h>
#include <conf_can.h>
#include <sercom_interrupt.h>  /* _sercom_set_handler(), _sercom_get_sercom_inst_index() */

/* =========================================================================
 * ASF CAN driver compatibility patch
 * Missing from older ASF versions — values from SAMC21 datasheet §45.6.7
 * ========================================================================= */
#ifndef CAN_RX_FIFO_0_MESSAGE_LOST
#  define CAN_RX_FIFO_0_MESSAGE_LOST           (1UL << 3)
#endif

/* =========================================================================
 * Build-time assertions
 * ========================================================================= */
#define STATIC_ASSERT(cond, msg)  typedef char static_assert_##msg[(cond) ? 1 : -1]

/* =========================================================================
 * Configuration
 * ========================================================================= */
#define SLCAN_VERSION           "V0002"
#define SLCAN_SERIAL            "NC21FD"
#define UART_BAUD_RATE          460800
#define CMD_BUF_SIZE            256

/* Worst-case SLCAN frame: 'B'+8hex+2hex+128hex+'\r' = 140 bytes */
#define SLCAN_FRAME_MAX         140u

/* DMA TX pool depth.
 * Each slot holds one max-size SLCAN frame (140 bytes).
 * Depth = how many frames can be queued while DMAC is busy sending.
 * 32 slots × 140 bytes = 4480 bytes RAM (14 % of 32 KB SRAM).
 * At 460800 baud each 140-byte frame takes ~3.0 ms to drain.
 *
 * Sizing rationale — CAN-TP CF burst (ISO 15765-2):
 *   ECU sends BS=15 consecutive frames back-to-back (STmin=0x00).
 *   Inter-frame gap on CAN FD 8-byte @ 500k/2M ? 150 µs.
 *   15 CFs arrive in ~2.25 ms; pool drains 1 slot every 3.0 ms.
 *   With TX_POOL_DEPTH=8 the pool saturated at slot 8 and frames
 *   9-15 were silently dropped ? wrong SN ? CAN-TP abort.
 *   32 slots gives a 96 ms drain window — far exceeds any burst. */
#define TX_POOL_DEPTH           32u

/* DMA channel assignment */
#define DMA_CH_TX               0u

#ifndef CONF_CAN0_TX_BUFFER_NUM
#  define CONF_CAN0_TX_BUFFER_NUM  1u
#endif

/* MCAN RX FIFO 0 depth.
 * Default from Atmel START / conf_can.h is 1 or 4 — far too shallow for
 * CAN-TP CF bursts.  Each element = R0 + R1 + 64 data = 72 bytes stored in
 * MCAN Message RAM (not in CPU SRAM).  64 × 72 = 4608 bytes Message RAM.
 * 64 is the hardware maximum on SAMC21 MCAN.
 * This guards against any main-loop latency spike (e.g. simultaneous
 * simulation TX) causing FIFO_MESSAGE_LOST during a CF burst. */
#ifndef CONF_CAN0_RX_FIFO_0_NUM
#  define CONF_CAN0_RX_FIFO_0_NUM  64u
#endif

STATIC_ASSERT(TX_POOL_DEPTH >= 2u,                 pool_needs_2_or_more_slots);
STATIC_ASSERT(CONF_CAN0_TX_BUFFER_NUM >= 1u,       need_at_least_one_tx_buffer);

/* =========================================================================
 * CAN FD DLC table
 * ========================================================================= */
static const uint8_t dlc_to_bytes[16] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8,
    12, 16, 20, 24, 32, 48, 64
};

/* =========================================================================
 * Module-level state
 * ========================================================================= */
static struct usart_module  cdc_instance;
static struct can_module    can_instance;
static struct can_rx_element_fifo_0 rx_element_fifo_0;

static volatile bool     can_rx_flag        = false;
static          bool     can_channel_open   = false;

static volatile uint16_t uart_overrun_count  = 0u;
static volatile uint16_t can_fifo_ovfl_count = 0u;

/* UART RX ring buffer (interrupt-driven).
 *
 * Root cause of corrupt CAN TX frames under high PCAN load:
 *   SERCOM USART has a 1-byte hardware RX buffer — no FIFO.
 *   A new byte arrives every 86.8µs at 115200 baud.
 *   can_process_rx() drains up to 8 CAN frames per call (~160µs).
 *   During that window, 1-2 RX bytes are overwritten (BUFOVF silently set).
 *   The Python TX command "B1C44001D08...
" arrives with holes.
 *   SLCAN parser gets a garbled ID ? can_send_fd() fires with wrong ID.
 *   That corrupt frame goes onto the CAN bus ? PCAN sees it.
 *
 * Fix: SERCOM RXC interrupt stores every byte into a 256-byte ring buffer.
 *      poll_uart_rx() drains the ring instead of reading SERCOM DATA directly.
 *      No bytes are ever lost regardless of main loop timing.
 */
#define UART_RX_RING_SIZE   256u
#define UART_RX_RING_MASK   (UART_RX_RING_SIZE - 1u)

STATIC_ASSERT((UART_RX_RING_SIZE & UART_RX_RING_MASK) == 0u, rx_ring_power_of_2);

static uint8_t          uart_rx_ring[UART_RX_RING_SIZE];
static volatile uint8_t uart_rx_w = 0u;   /* written by RXC ISR  */
static          uint8_t uart_rx_r = 0u;   /* read by main loop   */

/* UART RX command processing buffers — unchanged from v2 */
static          char     rx_buf[CMD_BUF_SIZE];
static volatile uint16_t rx_idx    = 0u;
static          char     proc_buf[CMD_BUF_SIZE];
static volatile bool     cmd_ready = false;

static volatile uint32_t system_tick_ms = 0u;

/* =========================================================================
 * SysTick
 * ========================================================================= */
void SysTick_Handler(void) { system_tick_ms++; }
static void configure_systick(void) { SysTick_Config(48000UL); }
static inline uint32_t millis(void) { return system_tick_ms; }
static inline void delay_ms(uint32_t ms)
{
    uint32_t t = millis();
    while ((millis() - t) < ms);
}

/* =========================================================================
 * CAN FD helpers
 * ========================================================================= */
static uint8_t bytes_to_dlc(uint8_t n)
{
    if (n <=  8u) return n;
    if (n <= 12u) return 9u;
    if (n <= 16u) return 10u;
    if (n <= 20u) return 11u;
    if (n <= 24u) return 12u;
    if (n <= 32u) return 13u;
    if (n <= 48u) return 14u;
    return 15u;
}

/* =========================================================================
 * DMA TX subsystem
 *
 * Replaces the ring buffer + DRE ISR + uart_tx_poll() from v2.
 *
 * Memory layout:
 *   tx_pool[TX_POOL_DEPTH][SLCAN_FRAME_MAX]  — frame storage
 *   tx_pool_len[TX_POOL_DEPTH]               — byte count per slot
 *   tx_q_head  — next slot for CPU to write into  (main loop only)
 *   tx_q_tail  — next slot for DMAC to read from  (ISR only)
 *   tx_q_count — slots currently occupied          (shared, IRQ-guarded)
 *
 * Invariant: tx_q_head != tx_q_tail unless tx_q_count == 0.
 * DMAC always reads from tail. CPU always writes to head.
 * They never access the same slot simultaneously.
 * ========================================================================= */
static uint8_t  tx_pool[TX_POOL_DEPTH][SLCAN_FRAME_MAX];
static uint8_t  tx_pool_len[TX_POOL_DEPTH];

static volatile uint8_t tx_q_head   = 0u;
static volatile uint8_t tx_q_tail   = 0u;
static volatile uint8_t tx_q_count  = 0u;
static volatile uint8_t tx_dma_busy = 0u;

static struct dma_resource          dma_tx_resource;
static struct dma_descriptor_config dma_tx_desc_cfg;
COMPILER_ALIGNED(16)
static DmacDescriptor               dma_tx_descriptor;

/* Forward declaration */
static bool dma_tx_submit(const uint8_t *buf, uint8_t len);

/* Start a DMA transfer for the frame currently at tx_q_tail.
 * Must only be called when DMAC channel is idle. */
static void dma_tx_kick(void)
{
    uint8_t slot = tx_q_tail;

    dma_descriptor_get_config_defaults(&dma_tx_desc_cfg);
    dma_tx_desc_cfg.beat_size              = DMA_BEAT_SIZE_BYTE;
    dma_tx_desc_cfg.src_increment_enable   = true;
    dma_tx_desc_cfg.dst_increment_enable   = false;
    dma_tx_desc_cfg.block_transfer_count   = tx_pool_len[slot];
    /* ASF end-address convention: point past last byte */
    dma_tx_desc_cfg.source_address         =
        (uint32_t)(tx_pool[slot] + tx_pool_len[slot]);
    dma_tx_desc_cfg.destination_address    =
        (uint32_t)(&cdc_instance.hw->USART.DATA.reg);
    dma_tx_desc_cfg.next_descriptor_address = 0u;   /* stop after this block */

    dma_descriptor_create(&dma_tx_descriptor, &dma_tx_desc_cfg);
    dma_start_transfer_job(&dma_tx_resource);
}

/* DMA transfer-complete callback.
 * Fires once per frame (not per byte) after all bytes have been sent.
 * Advances tail, starts next frame if the queue is non-empty. */
static void dma_tx_complete_cb(struct dma_resource *res)
{
    (void)res;

    /* Advance tail pointer */
    tx_q_tail = (tx_q_tail + 1u) % TX_POOL_DEPTH;

    /* Decrement count (shared) */
    __disable_irq();
    uint8_t remaining = --tx_q_count;
    __enable_irq();

    if (remaining > 0u) {
        /* More frames queued — start the next one immediately */
        dma_tx_kick();
    } else {
        tx_dma_busy = 0u;
    }
}

/* Initialise DMAC channel 0 for UART TX.
 * Called once from main(), after configure_usart_cdc(). */
static void dma_tx_init(void)
{
    struct dma_resource_config cfg;
    dma_get_config_defaults(&cfg);

    /* Trigger: SERCOM DRE (Data Register Empty).
     * Fires once per byte — DMAC writes one byte per trigger. */
    cfg.peripheral_trigger = EDBG_CDC_SERCOM_DMAC_ID_TX;
    cfg.trigger_action     = DMA_TRIGGER_ACTION_BEAT;
    cfg.priority           = DMA_PRIORITY_LEVEL_0;

    dma_allocate(&dma_tx_resource, &cfg);
    dma_add_descriptor(&dma_tx_resource, &dma_tx_descriptor);
    dma_register_callback(&dma_tx_resource, dma_tx_complete_cb,
                          DMA_CALLBACK_TRANSFER_DONE);
    dma_enable_callback(&dma_tx_resource, DMA_CALLBACK_TRANSFER_DONE);
}

/**
 * dma_tx_submit() — Queue a frame for DMA transmission.
 *
 * Copies buf[0..len-1] into the next free slot in tx_pool[].
 * If DMAC is idle, kicks off the transfer immediately.
 * If DMAC is busy, the frame will be sent when the current one finishes.
 * If the pool is full, the frame is dropped (uart_overrun_count++).
 *
 * Safe to call from main loop only (not from ISR).
 *
 * @param  buf  Frame bytes
 * @param  len  Byte count (1..SLCAN_FRAME_MAX)
 * @return true  Frame accepted
 * @return false Frame dropped (pool full)
 */
static bool dma_tx_submit(const uint8_t *buf, uint8_t len)
{
    if (len == 0u || len > SLCAN_FRAME_MAX) return false;

    __disable_irq();
    uint8_t count = tx_q_count;
    __enable_irq();

    if (count >= TX_POOL_DEPTH) {
        uart_overrun_count++;
        return false;
    }

    /* Copy frame into head slot */
    uint8_t slot = tx_q_head;
    memcpy(tx_pool[slot], buf, len);
    tx_pool_len[slot] = len;

    /* Advance head and increment count atomically */
    __disable_irq();
    tx_q_head  = (tx_q_head + 1u) % TX_POOL_DEPTH;
    tx_q_count++;
    uint8_t was_busy = tx_dma_busy;
    if (!was_busy) tx_dma_busy = 1u;
    __enable_irq();

    if (!was_busy) {
        /* DMAC was idle — start immediately */
        dma_tx_kick();
    }
    /* If DMAC was busy, dma_tx_complete_cb() will pick up the new slot */

    return true;
}

/* =========================================================================
 * Frame builder helpers  (identical to v2)
 * ========================================================================= */
typedef struct {
    uint8_t buf[SLCAN_FRAME_MAX];
    uint8_t len;
} FrameBuf;

static inline void fb_init(FrameBuf *f) { f->len = 0u; }

static inline void fb_putc(FrameBuf *f, char c)
{
    if (f->len < SLCAN_FRAME_MAX) f->buf[f->len++] = (uint8_t)c;
}

static void fb_hex_nibble(FrameBuf *f, uint8_t n)
{
    fb_putc(f, (n < 10u) ? (char)('0' + n) : (char)('A' + n - 10u));
}

static void fb_hex8(FrameBuf *f, uint8_t v)
{
    fb_hex_nibble(f, (v >> 4u) & 0x0Fu);
    fb_hex_nibble(f, v & 0x0Fu);
}

static void fb_hexN(FrameBuf *f, uint32_t val, uint8_t digits)
{
    for (int8_t i = (int8_t)(digits - 1); i >= 0; i--)
        fb_hex_nibble(f, (uint8_t)((val >> ((uint8_t)i * 4u)) & 0x0Fu));
}

/* Short response helper — wraps dma_tx_submit for 1-4 byte responses */
static void uart_putc_nb(char c)
{
    uint8_t b = (uint8_t)c;
    /* Spin briefly if pool is momentarily full (rare for short responses) */
    uint16_t guard = 10000u;
    while (guard-- > 0u) {
        __disable_irq();
        uint8_t full = (tx_q_count >= TX_POOL_DEPTH);
        __enable_irq();
        if (!full) break;
    }
    dma_tx_submit(&b, 1u);
}

static void uart_puts_nb(const char *s)
{
    /* Bundle entire string as one DMA submission for efficiency */
    uint8_t tmp[32];
    uint8_t n = 0u;
    while (*s && n < sizeof(tmp)) tmp[n++] = (uint8_t)*s++;
    if (n > 0u) dma_tx_submit(tmp, n);
}

/* =========================================================================
 * UART RX — interrupt-driven ring buffer + main-loop drainer
 * ========================================================================= */

/**
 * uart_rxc_sercom_handler() — ASF SERCOM per-instance RXC handler.
 *
 * ASF defines SERCOM3_Handler() in sercom_interrupt.c and dispatches to
 * a registered per-instance callback via _sercom_set_handler(). Defining
 * our own SERCOM3_Handler() causes a multiple-definition linker error.
 *
 * Fix: register this function via _sercom_set_handler(SERCOM_INST_NUM, ...)
 * in configure_usart_cdc(). ASF's SERCOM3_Handler calls _sercom_interrupt_
 * handler(3) which calls this function for every SERCOM3 interrupt.
 *
 * Signature matches sercom_handler_t: void fn(uint8_t instance).
 *
 * We own the entire SERCOM3 interrupt here. ASF's usart driver registered
 * a DRE callback for TX, but we replaced TX with DMA — DRE is never enabled,
 * so ASF's TX callback never fires. We only need to handle RXC.
 */
static void uart_rxc_sercom_handler(uint8_t instance)
{
    (void)instance;

    if (EDBG_CDC_MODULE->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_RXC) {
        uint8_t byte  = (uint8_t)(EDBG_CDC_MODULE->USART.DATA.reg);
        uint8_t next_w = (uart_rx_w + 1u) & UART_RX_RING_MASK;
        if (next_w != uart_rx_r) {      /* drop only if ring full (never in practice) */
            uart_rx_ring[uart_rx_w] = byte;
            uart_rx_w = next_w;
        }
        /* Clear BUFOVF — silently set if a byte arrived before ISR ran */
        EDBG_CDC_MODULE->USART.STATUS.reg = SERCOM_USART_STATUS_BUFOVF;
    }
}

/**
 * poll_uart_rx() — drain RX ring into command buffer.
 *
 * Reads characters from the ISR-filled ring buffer.
 * No direct SERCOM DATA register access here — ISR owns that.
 * Called from main loop; safe to call frequently with zero cost when ring empty.
 */
static void poll_uart_rx(void)
{
    while (uart_rx_r != uart_rx_w) {
        char c = (char)uart_rx_ring[uart_rx_r];
        uart_rx_r = (uart_rx_r + 1u) & UART_RX_RING_MASK;

        if (c == '\r' || c == '\n') {
            if (rx_idx > 0u && !cmd_ready) {
                memcpy(proc_buf, rx_buf, rx_idx);
                proc_buf[rx_idx] = '\0';
                cmd_ready = true;
                rx_idx = 0u;
            }
        } else if (rx_idx < (CMD_BUF_SIZE - 1u)) {
            rx_buf[rx_idx++] = c;
        }
    }
}

/* =========================================================================
 * USART / CDC configuration  (identical to v2)
 * ========================================================================= */
static void configure_usart_cdc(void)
{
    struct usart_config cfg;
    usart_get_config_defaults(&cfg);
    cfg.baudrate    = UART_BAUD_RATE;
    cfg.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
    cfg.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
    cfg.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
    cfg.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
    cfg.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;
    stdio_serial_init(&cdc_instance, EDBG_CDC_MODULE, &cfg);
    usart_enable(&cdc_instance);

    /* Register our RXC handler via ASF's SERCOM dispatch table.
     *
     * ASF's SERCOM3_Handler (in sercom_interrupt.c) calls the function
     * registered for instance 3 via _sercom_set_handler(). This avoids
     * redefining SERCOM3_Handler (which causes a linker conflict).
     *
     * _sercom_get_sercom_inst_index() returns the instance index (0-5).
     * Alternatively pass the literal 3 for SERCOM3 / EDBG CDC on
     * SAMC21 Xplained Pro.                                               */
    _sercom_set_handler(_sercom_get_sercom_inst_index(EDBG_CDC_MODULE),
                        uart_rxc_sercom_handler);
    EDBG_CDC_MODULE->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC;
    NVIC_EnableIRQ(SERCOM3_IRQn);
}

/* =========================================================================
 * CAN FD hardware configuration  (identical to v2)
 * ========================================================================= */
static void configure_can_fd(void)
{
    struct system_pinmux_config pin;
    system_pinmux_get_config_defaults(&pin);

    pin.mux_position = CAN_TX_MUX_SETTING;
    system_pinmux_pin_set_config(CAN_TX_PIN, &pin);

    pin.mux_position = CAN_RX_MUX_SETTING;
    system_pinmux_pin_set_config(CAN_RX_PIN, &pin);

    struct can_config cfg;
    can_get_config_defaults(&cfg);
    cfg.run_in_standby                      = false;
    cfg.nonmatching_frames_action_standard  = CAN_NONMATCHING_FRAMES_FIFO_0;
    cfg.nonmatching_frames_action_extended  = CAN_NONMATCHING_FRAMES_FIFO_0;
    cfg.protocol_exception_handling         = true;

    can_init(&can_instance, CAN_MODULE, &cfg);

    /* Enter init + CCE to write protected registers */
    CAN0->CCCR.reg |= CAN_CCCR_INIT;
    while (!(CAN0->CCCR.reg & CAN_CCCR_INIT));
    CAN0->CCCR.reg |= CAN_CCCR_CCE;

    /* CRITICAL: Disable self-reception.
     * MCAN by default loops TX frames back into RX FIFO0 (self-reception).
     * This causes every frame SAMC21 transmits to also appear as a received
     * frame, producing phantom/corrupted IDs in python-can.
     *
     * The ASF can_config struct in older versions lacks 'enable_self_reception'.
     * We fix it directly: clear TEST.LBCK (no internal loopback) and ensure
     * CCCR.TEST=0, CCCR.MON=0 — normal external bus mode.
     * With LBCK=0, MCAN does NOT store transmitted frames in RX buffers.   */
    CAN0->TEST.reg  &= ~CAN_TEST_LBCK;                    /* no TX?RX loopback */
    CAN0->CCCR.reg  &= ~(CAN_CCCR_TEST | CAN_CCCR_MON);  /* normal bus mode   */

    /* Nominal bit timing (500 kbit/s) */
    CAN0->NBTP.reg =
        CAN_NBTP_NBRP(CONF_CAN0_NBTP_NBRP_VALUE)    |
        CAN_NBTP_NTSEG1(CONF_CAN0_NBTP_NTSEG1_VALUE) |
        CAN_NBTP_NTSEG2(CONF_CAN0_NBTP_NTSEG2_VALUE) |
        CAN_NBTP_NSJW(CONF_CAN0_NBTP_NSJW_VALUE);

    /* Data bit timing (2 Mbit/s) + TDC */
    CAN0->DBTP.reg =
        CAN_DBTP_DBRP(CONF_CAN0_DBTP_DBRP_VALUE)    |
        CAN_DBTP_DTSEG1(CONF_CAN0_DBTP_DTSEG1_VALUE) |
        CAN_DBTP_DTSEG2(CONF_CAN0_DBTP_DTSEG2_VALUE) |
        CAN_DBTP_DSJW(CONF_CAN0_DBTP_DSJW_VALUE)    |
        CAN_DBTP_TDC;

    CAN0->TDCR.reg =
        CAN_TDCR_TDCO(CONF_CAN0_TDCR_TDCO_VALUE) |
        CAN_TDCR_TDCF(0u);

    /* Enable CAN FD + Bit Rate Switching */
    CAN0->CCCR.reg |= CAN_CCCR_FDOE | CAN_CCCR_BRSE;

    /* Leave init (CCE auto-clears) */
    CAN0->CCCR.reg &= ~CAN_CCCR_INIT;
    while (CAN0->CCCR.reg & CAN_CCCR_INIT);

    can_start(&can_instance);

    system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_CAN0);
    can_enable_interrupt(&can_instance,
        CAN_PROTOCOL_ERROR_ARBITRATION |
        CAN_PROTOCOL_ERROR_DATA        |
        CAN_RX_FIFO_0_MESSAGE_LOST);
}

static void can_open_channel(void)
{
    if (!can_channel_open) {
        can_enable_interrupt(&can_instance, CAN_RX_FIFO_0_NEW_MESSAGE);
        can_channel_open = true;
    }
}

static void can_close_channel(void)
{
    if (can_channel_open) {
        can_disable_interrupt(&can_instance, CAN_RX_FIFO_0_NEW_MESSAGE);
        can_channel_open = false;
    }
}

/* =========================================================================
 * TX buffer management  (identical to v2)
 * ========================================================================= */
static int8_t acquire_tx_buffer(void)
{
    static uint8_t next_buf = 0u;

    for (uint8_t a = 0u; a < CONF_CAN0_TX_BUFFER_NUM; a++) {
        uint8_t  idx  = (next_buf + a) % CONF_CAN0_TX_BUFFER_NUM;
        uint32_t mask = (1u << idx);
        if (!(CAN0->TXBRP.reg & mask)) {
            next_buf = (idx + 1u) % CONF_CAN0_TX_BUFFER_NUM;
            return (int8_t)idx;
        }
    }

    uint32_t deadline = millis() + 2u;
    while (millis() < deadline) {
        for (uint8_t idx = 0u; idx < CONF_CAN0_TX_BUFFER_NUM; idx++) {
            if (!(CAN0->TXBRP.reg & (1u << idx))) {
                next_buf = (idx + 1u) % CONF_CAN0_TX_BUFFER_NUM;
                return (int8_t)idx;
            }
        }
    }
    return -1;
}

static bool can_tx_commit(struct can_tx_element *te, int8_t buf)
{
    if (buf < 0) return false;
    can_set_tx_buffer_element(&can_instance, te, (uint8_t)buf);
    __DMB();
    __DSB();
    can_tx_transfer_request(&can_instance, (1u << (uint8_t)buf));
    port_pin_toggle_output_level(LED_0_PIN);
    return true;
}

static void can_send_classic(uint32_t id, bool extended,
                              const uint8_t *data, uint8_t dlc)
{
    int8_t buf = acquire_tx_buffer();
    if (buf < 0) return;

    struct can_tx_element te;
    can_get_tx_buffer_element_defaults(&te);
    te.T0.reg = extended
        ? (CAN_TX_ELEMENT_T0_EXTENDED_ID(id) | CAN_TX_ELEMENT_T0_XTD)
        :  CAN_TX_ELEMENT_T0_STANDARD_ID(id);
    te.T1.bit.DLC = dlc;
    te.T1.bit.FDF = 0;
    te.T1.bit.BRS = 0;
    memcpy(te.data, data, dlc);
    can_tx_commit(&te, buf);
}

static void can_send_fd(uint32_t id, bool extended,
                        const uint8_t *data, uint8_t num_bytes, bool brs)
{
    int8_t buf = acquire_tx_buffer();
    if (buf < 0) return;

    struct can_tx_element te;
    can_get_tx_buffer_element_defaults(&te);
    te.T0.reg = extended
        ? (CAN_TX_ELEMENT_T0_EXTENDED_ID(id) | CAN_TX_ELEMENT_T0_XTD)
        :  CAN_TX_ELEMENT_T0_STANDARD_ID(id);

    uint8_t dlc        = bytes_to_dlc(num_bytes);
    uint8_t padded_len = dlc_to_bytes[dlc];
    te.T1.bit.DLC = dlc;
    te.T1.bit.FDF = 1;
    te.T1.bit.BRS = brs ? 1u : 0u;
    memcpy(te.data, data, num_bytes);
    if (padded_len > num_bytes)
        memset(te.data + num_bytes, 0x00u, padded_len - num_bytes);
    can_tx_commit(&te, buf);
}

/* =========================================================================
 * CAN ISR  (identical to v2)
 * ========================================================================= */
void CAN0_Handler(void)
{
    uint32_t status = can_read_interrupt_status(&can_instance);

    if (status & CAN_RX_FIFO_0_NEW_MESSAGE) {
        can_clear_interrupt_status(&can_instance, CAN_RX_FIFO_0_NEW_MESSAGE);
        can_rx_flag = true;
    }

    if (status & CAN_RX_FIFO_0_MESSAGE_LOST) {
        can_clear_interrupt_status(&can_instance, CAN_RX_FIFO_0_MESSAGE_LOST);
        CAN0->RXF0A.reg = CAN_RXF0A_F0AI(
            (CAN0->RXF0S.reg & CAN_RXF0S_F0GI_Msk) >> CAN_RXF0S_F0GI_Pos);
        can_fifo_ovfl_count++;
        can_rx_flag = true;
    }

    if (status & (CAN_PROTOCOL_ERROR_ARBITRATION | CAN_PROTOCOL_ERROR_DATA)) {
        can_clear_interrupt_status(&can_instance,
            CAN_PROTOCOL_ERROR_ARBITRATION | CAN_PROTOCOL_ERROR_DATA);
    }
}

/* =========================================================================
 * CAN RX processing
 *
 * Identical to v2 except:
 *   uart_commit_frame(f.buf, f.len)  ?  dma_tx_submit(f.buf, f.len)
 *   uart_tx_poll() call removed      (DMAC drains concurrently, no poll needed)
 * ========================================================================= */
static void can_process_rx(void)
{
    if (!can_rx_flag || !can_channel_open) return;
    can_rx_flag = false;

    uint32_t fifo_status = CAN0->RXF0S.reg;
    uint8_t  fill = (uint8_t)((fifo_status & CAN_RXF0S_F0FL_Msk) >> CAN_RXF0S_F0FL_Pos);

    while (fill > 0u) {
        uint8_t gi = (uint8_t)((fifo_status & CAN_RXF0S_F0GI_Msk) >> CAN_RXF0S_F0GI_Pos);

        if (gi >= CONF_CAN0_RX_FIFO_0_NUM) break;

        can_get_rx_fifo_0_element(&can_instance, &rx_element_fifo_0, gi);
        can_rx_fifo_acknowledge(&can_instance, 0u, gi);

        port_pin_toggle_output_level(LED_0_PIN);

        bool is_ext = rx_element_fifo_0.R0.bit.XTD;
        bool is_rtr = rx_element_fifo_0.R0.bit.RTR;
        bool is_fd  = rx_element_fifo_0.R1.bit.FDF;
        bool is_brs = rx_element_fifo_0.R1.bit.BRS;

        uint32_t id = is_ext
            ? rx_element_fifo_0.R0.bit.ID
            : (rx_element_fifo_0.R0.bit.ID >> 18u);

        uint8_t dlc = rx_element_fifo_0.R1.bit.DLC;
        if (dlc > 15u) dlc = 15u;
        uint8_t num_bytes = is_fd ? dlc_to_bytes[dlc] : (uint8_t)dlc;

        char ftype;
        if (is_fd) {
            ftype = is_brs ? (is_ext ? 'B' : 'b') : (is_ext ? 'D' : 'd');
        } else {
            ftype = is_rtr ? (is_ext ? 'R' : 'r') : (is_ext ? 'T' : 't');
        }

        /* Build SLCAN frame atomically in local stack buffer */
        FrameBuf f;
        fb_init(&f);
        fb_putc(&f, ftype);
        fb_hexN(&f, id, is_ext ? 8u : 3u);

        if (is_fd) {
            fb_hex8(&f, num_bytes);
        } else {
            fb_putc(&f, (char)('0' + dlc));
        }

        if (!is_rtr) {
            for (uint8_t i = 0u; i < num_bytes; i++)
                fb_hex8(&f, rx_element_fifo_0.data[i]);
        }

        fb_putc(&f, '\r');

        /* Submit to DMA pool — non-blocking, atomic, no partial frames */
        dma_tx_submit(f.buf, f.len);

        /* Refresh fill level */
        fifo_status = CAN0->RXF0S.reg;
        fill = (uint8_t)((fifo_status & CAN_RXF0S_F0FL_Msk) >> CAN_RXF0S_F0FL_Pos);
    }
}

/* =========================================================================
 * SLCAN command parser  (identical to v2)
 * ========================================================================= */
static int8_t hexc(char c)
{
    if (c >= '0' && c <= '9') return (int8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (int8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return (int8_t)(c - 'a' + 10);
    return -1;
}

static uint32_t hex_parse(const char *s, uint8_t ndig)
{
    uint32_t v = 0u;
    for (uint8_t i = 0u; i < ndig; i++) {
        int8_t n = hexc(s[i]);
        if (n < 0) return 0u;
        v = (v << 4u) | (uint8_t)n;
    }
    return v;
}

static bool hex_parse_bytes(const char *buf, uint8_t *out, uint8_t n)
{
    for (uint8_t i = 0u; i < n; i++) {
        int8_t hi = hexc(buf[i * 2u]);
        int8_t lo = hexc(buf[i * 2u + 1u]);
        if (hi < 0 || lo < 0) return false;
        out[i] = (uint8_t)((hi << 4) | lo);
    }
    return true;
}

static void slcan_process_command(void)
{
    if (!cmd_ready) return;

    char    cmd = proc_buf[0];
    uint8_t len = (uint8_t)strlen(proc_buf);
    bool    ok  = true;

    switch (cmd) {

    case 'V':
        uart_puts_nb(SLCAN_VERSION);
        uart_putc_nb('\r');
        break;

    case 'N':
        uart_puts_nb(SLCAN_SERIAL);
        uart_putc_nb('\r');
        break;

    case 'O':
        can_open_channel();
        break;

    case 'C':
        can_close_channel();
        break;

    case 'S':
        break;

    case 'F': {
        uint8_t  flags = 0u;
        uint32_t psr   = CAN0->PSR.reg;
        if (psr & CAN_PSR_BO)          flags |= 0x80u;
        if (psr & CAN_PSR_EP)          flags |= 0x20u;
        if (can_fifo_ovfl_count > 0u)  flags |= 0x08u;
        if (psr & CAN_PSR_EW)          flags |= 0x04u;
        if (uart_overrun_count > 0u)   flags |= 0x02u;

        { /* scope for hi/lo nibble locals */
            uint8_t hi = (flags >> 4u)  & 0x0Fu;
            uint8_t lo =  flags         & 0x0Fu;
            uart_putc_nb('F');
            uart_putc_nb((hi < 10u) ? (char)('0' + hi) : (char)('A' + hi - 10u));
            uart_putc_nb((lo < 10u) ? (char)('0' + lo) : (char)('A' + lo - 10u));
            uart_putc_nb('\r');
        }

        uart_overrun_count  = 0u;
        can_fifo_ovfl_count = 0u;
        break;
    }

    case 't': case 'T': {
        bool    ext     = (cmd == 'T');
        uint8_t id_dig  = ext ? 8u : 3u;
        uint8_t hdr_len = 1u + id_dig + 1u;

        if (len < hdr_len) { ok = false; break; }

        uint32_t id  = hex_parse(&proc_buf[1], id_dig);
        int8_t   dlc = hexc(proc_buf[1u + id_dig]);

        if (dlc < 0 || dlc > 8) { ok = false; break; }
        if (len < hdr_len + (uint8_t)((uint8_t)dlc * 2u)) { ok = false; break; }

        uint8_t data[8] = {0};
        if (dlc > 0 && !hex_parse_bytes(&proc_buf[hdr_len], data, (uint8_t)dlc)) {
            ok = false; break;
        }
        can_send_classic(id, ext, data, (uint8_t)dlc);
        break;
    }

    case 'r': case 'R':
        ok = false;
        break;

    case 'd': case 'b': case 'D': case 'B': {
        bool ext = (cmd == 'D' || cmd == 'B');
        bool brs = (cmd == 'b' || cmd == 'B');

        uint8_t id_dig  = ext ? 8u : 3u;
        uint8_t hdr_len = 1u + id_dig + 2u;

        if (len < hdr_len) { ok = false; break; }

        uint32_t id        = hex_parse(&proc_buf[1], id_dig);
        uint32_t num_bytes = hex_parse(&proc_buf[1u + id_dig], 2u);

        if (num_bytes > 64u) { ok = false; break; }
        if (len < hdr_len + (uint8_t)(num_bytes * 2u)) { ok = false; break; }

        uint8_t data[64] = {0};
        if (num_bytes > 0u && !hex_parse_bytes(&proc_buf[hdr_len], data, (uint8_t)num_bytes)) {
            ok = false; break;
        }
        can_send_fd(id, ext, data, (uint8_t)num_bytes, brs);
        break;
    }

    default:
        ok = false;
        break;
    }

    if (ok) {
        if (cmd != 'V' && cmd != 'N' && cmd != 'F') uart_putc_nb('\r');
    } else {
        uart_putc_nb('\a');
    }

    cmd_ready = false;
}

/* =========================================================================
 * Main
 * ========================================================================= */
int main(void)
{
    system_init();
    system_interrupt_enable_global();

    configure_systick();
    configure_usart_cdc();

    /* DMA init AFTER USART — needs cdc_instance.hw pointer */
    dma_tx_init();

    configure_can_fd();

    /* 5× blink at startup */
    for (uint8_t i = 0u; i < 5u; i++) {
        port_pin_set_output_level(LED_0_PIN, LED_0_ACTIVE);
        delay_ms(100u);
        port_pin_set_output_level(LED_0_PIN, !LED_0_ACTIVE);
        delay_ms(100u);
    }
    port_pin_set_output_level(LED_0_PIN, LED_0_ACTIVE);

    while (1) {
        poll_uart_rx();
        slcan_process_command();
        can_process_rx();
        /* No uart_tx_poll() needed — DMAC drains TX autonomously */
    }
}