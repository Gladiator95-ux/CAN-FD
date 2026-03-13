"""
initialize_pcan.py
Handles CAN Channel initialization and closing with Hardware Selection (Vector/PEAK/SAMC21).
"""
import os
import sys
import can
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QPushButton, QHBoxLayout, 
    QFrame, QButtonGroup, QToolButton
)
from PySide6.QtCore import Qt, QSize
from PySide6.QtGui import QIcon, QPixmap
from can.interfaces.pcan import PcanError
from can.interfaces.vector import VectorError

import styles.styles as styles
from server_manager import app_globals

# Import SAMC21 custom bus class
from .samc21_interface import SAMC21Bus

class INITIALIZE_PCAN_Page(QWidget):
    def __init__(self, base_directory):
        super().__init__()
        self.base_dir = base_directory
        self.selected_hardware = None  # Track selected hardware
        self.init_ui()

    def init_ui(self):
        self.layout = QVBoxLayout(self)
        self.layout.setAlignment(Qt.AlignmentFlag.AlignCenter)  # Center content vertically
        self.layout.setContentsMargins(40, 40, 40, 40)
        self.layout.setSpacing(20)

        # --- 1. Header Label ---
        main_header_label = QLabel("Multi-Hardware Functionality")
        main_header_label.setStyleSheet("""
            QLabel { font-size: 14px; font-weight: bold; color: #555; background-color: #e8e8e8; padding: 8px 15px; border-radius: 15px; }
        """)
        main_header_label.setAlignment(Qt.AlignmentFlag.AlignCenter)  # Center text horizontally
        self.layout.addWidget(main_header_label)

        # --- 2. Sub-header ---
        header_label = QLabel("Choose Hardware and Initialize Channel")
        header_label.setStyleSheet("font-size: 18px; color: #2c3e50;")
        header_label.setAlignment(Qt.AlignmentFlag.AlignCenter)  # Center text horizontally
        self.layout.addWidget(header_label)
        
        self.layout.addSpacing(10)

        # --- 3. Hardware Selection ---
        hw_layout = QHBoxLayout()
        hw_layout.setSpacing(30)
        hw_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)  # Center layout horizontally

        # Button Group for mutual exclusivity
        self.hw_group = QButtonGroup(self)
        self.hw_group.setExclusive(True)

        # -- Vector Hardware Button --
        self.btn_vector = self.create_hardware_button("Vector Hardware", "vector.png", icon_size=QSize(98, 98), padding_top=40)
        self.hw_group.addButton(self.btn_vector)
        hw_layout.addWidget(self.btn_vector)

        # -- PEAK Hardware Button --
        self.btn_peak = self.create_hardware_button("PEAK Hardware", "peak.png", icon_size=QSize(80, 80), padding_top=30)
        self.hw_group.addButton(self.btn_peak)
        hw_layout.addWidget(self.btn_peak)

        # -- SAMC21 Hardware Button --
        self.btn_samc21 = self.create_hardware_button("SAMC21 Hardware", "microchip_logo.png", icon_size=QSize(90, 90), padding_top=35)
        self.hw_group.addButton(self.btn_samc21)
        hw_layout.addWidget(self.btn_samc21)
        
        # Connect clicked signal to style update
        self.hw_group.buttonClicked.connect(self.on_hardware_selected)

        self.layout.addLayout(hw_layout)
        
        self.layout.addSpacing(30)

        # --- 4. Action Buttons ---
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(20)
        btn_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)  # Center layout horizontally

        # Initialize Button
        self.set_btn = QPushButton("Initialize Channel")
        self.set_btn.setFixedWidth(200)
        self.set_btn.setStyleSheet(styles.TEST_BUTTON_STYLE)
        self.set_btn.clicked.connect(self.initialize_channel)
        
        # Close Button
        self.close_btn = QPushButton("Close Channel")
        self.close_btn.setFixedWidth(200)
        self.close_btn.setStyleSheet(styles.TEST_BUTTON_STYLE)
        self.close_btn.clicked.connect(self.close_channel)
        
        # Initially disable buttons
        self.close_btn.setEnabled(False)
        self.set_btn.setEnabled(False) # Enabled only after selection

        btn_layout.addWidget(self.set_btn)
        btn_layout.addWidget(self.close_btn)
        
        self.layout.addLayout(btn_layout)

        # --- 5. Status Message ---
        self.status_label = QLabel("")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)  # Center text horizontally
        self.status_label.setStyleSheet("font-size: 16px; margin-top: 20px; font-weight: bold;")
        self.layout.addWidget(self.status_label)
        
        # Removed addStretch to allow vertical centering by AlignCenter

    def create_hardware_button(self, text, icon_name, icon_size=QSize(90, 90), padding_top=35):
        """Helper to create stylized hardware selection buttons."""
        btn = QToolButton()
        btn.setText(text)
        btn.setCheckable(True)
        # Set size for a vertical box layout
        btn.setFixedSize(220, 180) 
        # Stack text under icon
        btn.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextUnderIcon)
        
        # Icon
        icon_path = os.path.join(self.base_dir, "images", icon_name)
        if os.path.exists(icon_path):
            btn.setIcon(QIcon(icon_path))
            btn.setIconSize(icon_size)
        else:
            # Fallback if image missing
            btn.setIconSize(QSize(0, 0))
            print(f"⚠️  Warning: Icon not found at {icon_path}")

        # Modern Style
        btn.setStyleSheet(f"""
            QToolButton {{
                background-color: #ffffff;
                border: 2px solid #bdc3c7;
                border-radius: 12px;
                color: #2c3e50;
                font-weight: bold;
                font-size: 14px;
                        
                /* 🔽 Push icon + text downward */
                padding-top: {padding_top}px;
                padding-bottom: 10px;
                padding-left: 15px;
                padding-right: 15px;
            }}
            QToolButton:hover {{
                border-color: #3498db;
                background-color: #f0f8ff;
            }}
            QToolButton:checked {{
                border: 3px solid #27ae60;
                background-color: #eafaf1;
                color: #27ae60;
            }}
        """)

        return btn

    def on_hardware_selected(self, button):
        """Handle hardware selection change."""
        if button == self.btn_vector:
            self.selected_hardware = "Vector"
        elif button == self.btn_peak:
            self.selected_hardware = "PEAK"
        elif button == self.btn_samc21:
            self.selected_hardware = "SAMC21"
        
        self.set_btn.setEnabled(True)
        self.status_label.setText(f"{self.selected_hardware} selected. Ready to initialize.")
        self.status_label.setStyleSheet("font-size: 16px; margin-top: 20px; font-weight: bold; color: #34495e;")

    def initialize_channel(self):
        """Handler for initialization based on selection."""
        if not self.selected_hardware:
            self.status_label.setText("Please select a hardware interface first.")
            self.status_label.setStyleSheet("font-size: 16px; margin-top: 20px; font-weight: bold; color: #e74c3c;")
            return

        self.status_label.setText(f"Initializing {self.selected_hardware}...")
        self.status_label.setStyleSheet("font-size: 16px; margin-top: 20px; font-weight: bold; color: #f39c12;") # Orange processing
        self.set_btn.setEnabled(False) # Prevent double click
        self.repaint() # Force UI update

        try:
            if self.selected_hardware == "Vector":
                app_globals.PCAN_HANDLE = self.initialize_vector_channel_in_background(
                    channel=0, 
                    bitrate=500000, 
                    fd=True, 
                    data_bitrate=2000000,
                    app_name="NovaCan"
                )
            elif self.selected_hardware == "PEAK":
                app_globals.PCAN_HANDLE = self.initialize_pcan_channel_in_background(
                    channel="PCAN_USBBUS1", 
                    bitrate=500000, 
                    fd=True, 
                    data_bitrate=2000000
                )
            elif self.selected_hardware == "SAMC21":
                app_globals.PCAN_HANDLE = self.initialize_samc21_channel_in_background(
                port=None,         
                baudrate=460800,
                auto_detect=True    
                )

            if app_globals.PCAN_HANDLE is None:
                raise ValueError("Bus handle is None! Please check hardware connection.")
            
            # Success
            self.status_label.setText(f"{self.selected_hardware} Initialized Successfully.")
            self.status_label.setStyleSheet("font-size: 16px; margin-top: 20px; font-weight: bold; color: #27ae60;")
            
            self.set_btn.setEnabled(False)
            self.close_btn.setEnabled(True)
            
            # Disable selection while running
            self.btn_vector.setEnabled(False)
            self.btn_peak.setEnabled(False)
            self.btn_samc21.setEnabled(False)

        except Exception as e:
            self.status_label.setText(f"Initialization Failed: {str(e)}")
            self.status_label.setStyleSheet("font-size: 16px; margin-top: 20px; font-weight: bold; color: #e74c3c;")
            self.set_btn.setEnabled(True)

    def close_channel(self):
        """Handler for closing the channel."""
        try:
            if hasattr(app_globals, 'PCAN_HANDLE') and app_globals.PCAN_HANDLE:
                app_globals.PCAN_HANDLE.shutdown()
                app_globals.PCAN_HANDLE = None
            
            self.status_label.setText("Channel Closed.")
            self.status_label.setStyleSheet("font-size: 16px; margin-top: 20px; font-weight: bold; color: #27ae60;")
            
            self.set_btn.setEnabled(True)
            self.close_btn.setEnabled(False)
            
            # Re-enable selection
            self.btn_vector.setEnabled(True)
            self.btn_peak.setEnabled(True)
            self.btn_samc21.setEnabled(True)

        except Exception as e:
            self.status_label.setText(f"Error Closing Channel: {str(e)}")
            self.status_label.setStyleSheet("font-size: 16px; margin-top: 20px; font-weight: bold; color: #e74c3c;")
            self.set_btn.setEnabled(True)
            self.close_btn.setEnabled(False)

    # --- Backend Functions ---

    def initialize_vector_channel_in_background(self, channel=0, bitrate=500000, fd=False, data_bitrate=2000000, app_name="CANalyzer"):
        """
        Initializes a Vector CAN channel with explicit FD bit timings to fix Stuff and CRC errors.
        """
        bus = None
        try:
            # 1. Define base parameters
            bus_params = {
                "interface": "vector",
                "channel": channel,
                "app_name": app_name,
                "fd": fd,
            }

            # 2. Add Robust Bit Timings for CAN FD
            # These values prevent "Stuff Errors" by explicitly defining the sample point.
            if fd:
                if not data_bitrate:
                    raise ValueError("For CAN FD, you must set data_bitrate")
                
                bus_params.update({
                    "bitrate": bitrate,
                    "data_bitrate": data_bitrate,
                    # Arbitration Phase (Nominal) Timings
                    "sjw_abr": 8,
                    "tseg1_abr": 63,
                    "tseg2_abr": 16,
                    # Data Phase (Fast) Timings - This fixes your CRC/Stuff errors
                    "sjw_dbr": 4,
                    "tseg1_dbr": 15,
                    "tseg2_dbr": 4,
                })
            else:
                # Standard CAN fallback
                bus_params["bitrate"] = bitrate

            # 3. Initialize the bus
            bus = can.Bus(**bus_params)
            print(f"[Vector] Channel {channel} initialized with manual timings ✅")

        except (VectorError, Exception) as e:
            print(f"[Vector] Initialization failed ❌: {e}")
            # Hint: If this still fails, check "ISO CAN FD" settings in Vector Hardware Config
        
        return bus
    
    

    def initialize_pcan_channel_in_background(self, channel="PCAN_USBBUS1", bitrate=500000, fd=False, data_bitrate=None):
        """
        Test if a PCAN channel can be initialized and then closed.
        """
        bus = None
        try:
            bus_params = {
                "bustype": "pcan",  # Specifies the CAN interface type, 'pcan' for PEAK-System PCAN devices
                "channel": channel,  # Defines the PCAN channel to use, e.g., 'PCAN_USBBUS1' for the first USB channel
                "bitrate": bitrate,  # Sets the nominal bitrate for standard CAN in bits per second (e.g., 500000 for 500 kbps)
                "f_clock_mhz": 80,  # Clock frequency of the CAN controller in MHz (80 MHz is standard for PCAN-USB FD)
                "nom_brp": 1,  # Nominal Bit Rate Prescaler, divides f_clock to set the time quantum for nominal bitrate
                "nom_tseg1": 127,  # Nominal Time Segment 1, number of time quanta before sampling (propagation + phase segment 1)
                "nom_tseg2": 32,  # Nominal Time Segment 2, number of time quanta after sampling (phase segment 2)
                "nom_sjw": 32,  # Nominal Synchronization Jump Width, max time quanta to adjust for clock drift in nominal phase
            }
            
            if fd:
                if not data_bitrate:
                    raise ValueError("For CAN FD, you must set data_bitrate")
                bus_params.update({
                    "fd": True,  # Enables CAN FD (Flexible Data Rate) mode for higher data rates and larger payloads
                    "data_bitrate": data_bitrate,  # Sets the data phase bitrate for CAN FD in bits per second (e.g., 2000000 for 2 Mbps)
                    "data_brp": 2,  # Data Bit Rate Prescaler, divides f_clock for data phase time quantum
                    "data_tseg1": 15,  # Data Time Segment 1, time quanta before sampling in data phase
                    "data_tseg2": 4,  # Data Time Segment 2, time quanta after sampling in data phase
                    "data_sjw": 4,  # Data Synchronization Jump Width, max time quanta for clock drift in data phase
                })
            
            bus = can.Bus(**bus_params)
            print("[PCAN] Channel initialized successfully ✅")
            
        except PcanError as e:
            print(f"[PCAN] Channel initialization failed ❌: PCAN Error Code: {e.error_code}, Description: {e}")
            return bus
        except Exception as e:
            print(f"[PCAN] Channel initialization failed ❌: {e}")
            return bus
        return bus

    def initialize_samc21_channel_in_background(self, port=None, baudrate=460800, auto_detect=True):
        """
        Initialize SAMC21 CAN channel via custom SAMC21Bus interface.
    
        Args:
        port: COM port (e.g., 'COM4'). If None, will auto-detect.
        baudrate: Serial baudrate (default: 115200)
        auto_detect: Enable automatic port detection
    
        Returns:
            SAMC21Bus instance or None on failure
        """
        bus = None
        try:
            # If port is None or "auto", enable auto-detection
            if port is None or port == "auto":
                print("[SAMC21] Auto-detecting COM port...")
                bus = SAMC21Bus(channel=None, baudrate=baudrate, auto_detect=True)
            else:
                print(f"[SAMC21] Using specified port: {port}")
                bus = SAMC21Bus(channel=port, baudrate=baudrate, auto_detect=False)
        
            print(f"[SAMC21] Channel initialized successfully ✅")
        
        except Exception as e:
            print(f"[SAMC21] Initialization failed ❌: {e}")
    
        return bus