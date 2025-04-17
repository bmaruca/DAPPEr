import sys
import serial
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget, QComboBox
from PyQt5.QtCore import QThread
from datetime import datetime

import serial
import struct
from PyQt5.QtCore import QThread, pyqtSignal
import packet_definitions

# THIS VALUE SHOULD BE UPDATED TO MATCH THE CURRENT PORTENTA PORT!!!
current_port = "COM6"

class SerialReader(QThread): # inherits thread to carry process out on (don't interrupt running GUI)
    dataReceived = pyqtSignal(dict)  # Parse data as dictionary

    def __init__(self, port=current_port, baudrate=9600):  # update port when connected to portenta/baudrate
        super().__init__()
        self.running = True
        try:
            self.serialPort = serial.Serial(port, baudrate, timeout=1)
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            self.serialPort = None

    def run(self):
        while self.running: 
            raw_packet = self.find_packet_start() # find start of packet in buffer, copies all following incoming bytes
            
            pkt_type_byte = raw_packet[5] if len(raw_packet) > 5 else None
            pkt_type_string = self.packet_type_byte_to_string(pkt_type_byte)

            # === HEADER/PACKET VALIDATION ===
            if len(raw_packet) == 0:
                print("No valid packet found") # find_packet_start() returns empty array if no packet found (no sync word)
            elif pkt_type_byte is None: 
                print("Invalid header") # NEED TO IMRPOVE DOESNT WORK RN
            elif pkt_type_string is None:
                print("Unknown packet type")
            elif len(raw_packet) != packet_definitions.PACKET_TYPES[pkt_type_string]["payload_size"] + 15: # add header bytes
                print("Incorrect packet length")
                print(len(raw_packet))
            elif raw_packet[2] != packet_definitions.PACKET_TYPES[pkt_type_string]["version"]:
                print("Incorrect packet version")
            else:
                # === UNPACK PAYLOAD DATA ===

                # UPDATE THIS WITH FC PACKET INFO
                # Unpack data into variables
                # do we want to unpack just the fc packet info or also the header info?
                # sync, version, checksum, pkt_type = struct.unpack("<HBHB", raw_packet[:6])
                if len(raw_packet) >= 35:
                    print(len(raw_packet))
                    sd_space, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, mag_x, mag_y, mag_z = struct.unpack("<HHHHHHHHHH", raw_packet[15:])
                else:
                    print("Error: Raw packet is too short for unpacking.")
                    print(len(raw_packet))
                # Handle the error or skip processing
                            # < = little endian
                            # H = 16-bit int (2 bytes) unsigned
                            # B = 8-bit int (1 byte) unsigned

                parsed_data = {
                    "sd_space": sd_space,
                    "gyro_x": gyro_x,
                    "gyro_y": gyro_y,
                    "gyro_z": gyro_z,
                    "accel_x": accel_x,
                    "accel_y": accel_y,
                    "accel_z": accel_z,
                    "mag_x": mag_x,
                    "mag_y": mag_y,
                    "mag_z": mag_z
                }
                
                # ADD CHECKSUM VALIDATION HERE ONCE CHECKSUM FUNCTION IS DEFINED
                
                self.dataReceived.emit(parsed_data)  # send to GUI
            
                # NOW, set up portenta to spit out dummy packet so you can test. like, every 5 seconds!
                # each UINT-16 increments by 1, so you can test sending/receiving.

                # GUI tab for each packet (add time for last packet received for debugging)

    def find_packet_start(self):
        buffer = bytearray() 
        raw_packet = bytearray() 
        max_attempts = 10000  # Limit the number of attempts to find a sync word
        attempts = 0

        while attempts < max_attempts:
            byte = self.serialPort.read(1)  
            
            # If no byte is read, print a message
            if not byte:
                print("No bytes received from the serial port.")
                continue  # Skip the rest of the loop and try reading again
            print("Byte received:", byte[0])
            buffer.append(byte[0])  # Append the first byte (byte[0]) as an integer

            # print("Buffer:", buffer)
            if len(buffer) > 1 and int(buffer[-2]) == 85 and int(buffer[-1]) == 68:
                print("Sync word found")
                sync_word_index = len(buffer) - 2 
                
                raw_packet.extend(buffer[sync_word_index:])  # Add the entire packet from sync word onwards

                # Continue collecting the remaining bytes from the serial buffer
                while self.serialPort.in_waiting > 0:  
                    byte = self.serialPort.read(1)  
                    if byte:  # Ensure byte is not empty before appending
                        raw_packet.append(byte[0])  # Append the byte as an integer
                return raw_packet
            else:
                print("Sync word not found")
        
        return bytearray()  # No packet (sync word) found / max attempts reached

    def packet_type_byte_to_string(self, packet_type_byte):
        for packet_name, packet_info in packet_definitions.PACKET_TYPES.items():
            if packet_info["packet_type"] == packet_type_byte:
                return packet_name
        return None
        

class GyroGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        self.packet_history = [] # Store past-received packets
        self.latest_packet = None

        self.setWindowTitle("Flight Computer Packet")

        self.label_sd_space = QLabel("SD Space: Waiting for data...")
        self.label_gyro_x = QLabel("Gyro X: Waiting for data...")
        self.label_gyro_y = QLabel("Gyro Y: Waiting for data...")
        self.label_gyro_z = QLabel("Gyro Z: Waiting for data...")
        self.label_accel_x = QLabel("Accel X: Waiting for data...")
        self.label_accel_y = QLabel("Accel Y: Waiting for data...")
        self.label_accel_z = QLabel("Accel Z: Waiting for data...")
        self.label_mag_x = QLabel("Mag X: Waiting for data...")
        self.label_mag_y = QLabel("Mag Y: Waiting for data...")
        self.label_mag_z = QLabel("Mag Z: Waiting for data...")

        # Dropdown for packet selection (live vs. history)
        self.packet_dropdown = QComboBox()
        self.packet_dropdown.addItem("Live Data")
        self.packet_dropdown.currentIndexChanged.connect(self.display_selected_packet)
        
        layout = QVBoxLayout()
        layout.addWidget(self.packet_dropdown)
        layout.addWidget(self.label_sd_space)
        layout.addWidget(self.label_gyro_x)
        layout.addWidget(self.label_gyro_y)
        layout.addWidget(self.label_gyro_z)
        layout.addWidget(self.label_accel_x)
        layout.addWidget(self.label_accel_y)
        layout.addWidget(self.label_accel_z)
        layout.addWidget(self.label_mag_x)
        layout.addWidget(self.label_mag_y)
        layout.addWidget(self.label_mag_z)

        centralWidget = QWidget()
        centralWidget.setLayout(layout)
        self.setCentralWidget(centralWidget)

        self.serialThread = SerialReader(current_port) 
        self.serialThread.dataReceived.connect(self.handle_new_packet)  # Connect to hande_new_packet method
        self.serialThread.start()

    def handle_new_packet(self, data):
        self.latest_packet = data
        self.packet_history.append(data)
        current_timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.packet_dropdown.addItem(f"Packet {len(self.packet_history)} ({current_timestamp})")

        if self.packet_dropdown.currentIndex() == 0:
            self.display_packet(data) # Live data

    def display_selected_packet(self, index):
        if index == 0 and self.latest_packet:
            self.display_packet(self.latest_packet)
        elif 1 <= index <= len(self.packet_history):
            self.display_packet(self.packet_history[index - 1])

    def display_packet(self, data):
        self.label_sd_space.setText(f"SD Space: {data['sd_space']}")
        self.label_gyro_x.setText(f"Gyro X: {data['gyro_x']}")
        self.label_gyro_y.setText(f"Gyro Y: {data['gyro_y']}")
        self.label_gyro_z.setText(f"Gyro Z: {data['gyro_z']}")
        self.label_accel_x.setText(f"Accel X: {data['accel_x']}")
        self.label_accel_y.setText(f"Accel Y: {data['accel_y']}")
        self.label_accel_z.setText(f"Accel Z: {data['accel_z']}")
        self.label_mag_x.setText(f"Mag X: {data['mag_x']}")
        self.label_mag_y.setText(f"Mag Y: {data['mag_y']}")
        self.label_mag_z.setText(f"Mag Z: {data['mag_z']}")

    def closeEvent(self, event):
        self.serialThread.stop()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = GyroGUI()
    window.show()
    sys.exit(app.exec_())