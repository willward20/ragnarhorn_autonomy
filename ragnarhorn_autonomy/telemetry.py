#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import serial
import threading
import struct
from struct import Struct
import time

from std_msgs.msg import String
from ragnarhorn_interfaces.msg import Telemetry

# -------------------------------------------
# Protocol structs
MsgHeader = Struct('<LBBB')
MsgTelem  = Struct('<QBHHHHHHHHBBBBHHHH')
MsgDrive  = Struct('<LBBBHH')

AUTONOMY_MAGIC   = 0x5A5A5A5A
AUTONOMY_VERSION = 0x01
AUTONOMY_TELEM   = 0x81

# -------------------------------------------
def crc8(data):
    c = 0
    for b in data:
        c ^= b
        for _ in range(8):
            if c & 0x80:
                c = (c << 1) ^ 0x1E7
            else:
                c <<= 1
    return c & 0xFF

# -------------------------------------------
class AutonomySerialInterface(threading.Thread):
    """
    Dedicated serial reader thread.
    Calls a callback every time a valid telemetry packet arrives.
    """

    def __init__(self, port, baudrate, telemetry_callback):
        super().__init__(daemon=True)
        self.telemetry_callback = telemetry_callback
        self.running = True

        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=None
        )

    def stop(self):
        self.running = False
        if self.ser.is_open:
            self.ser.close()

    def run(self):
        sync = False

        try:
            while self.running:
                header = bytearray()

                # ---- Sync on magic ----
                if sync:
                    header = self.ser.read(MsgHeader.size)
                else:
                    while not sync and self.running:
                        b = self.ser.read(1)
                        if b == b'Z':
                            header += b
                            if header == b'ZZZZ':
                                header += self.ser.read(MsgHeader.size - 4)
                                sync = True
                        else:
                            header = bytearray()

                if not self.running:
                    break

                # ---- Parse header ----
                magic, version, msg_type, msg_crc = MsgHeader.unpack(header)

                if magic != AUTONOMY_MAGIC or version != AUTONOMY_VERSION:
                    sync = False
                    continue

                # ---- Read body (fixed max size) ----
                body = self.ser.read(MsgTelem.size)

                # ---- CRC check ----
                crc_input = MsgHeader.pack(magic, version, msg_type, 0) + body
                if msg_crc != crc8(crc_input):
                    sync = False
                    continue

                # ---- Telemetry ----
                if msg_type == AUTONOMY_TELEM:
                    telem = MsgTelem.unpack(body)
                    self.telemetry_callback(telem)

        except Exception as e:
            print(f"[SerialThread] Error: {e}")

# -------------------------------------------
class AutonomyTelemetryNode(Node):
    """
    ROS 2 node wrapper.
    Does NOT publish yet.
    Provides access to latest telemetry and a per-sample callback.
    """

    def __init__(self):
        super().__init__('autonomy_telemetry_node')

        # Parameters (ROS-friendly)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 460800)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value

        self.get_logger().info(f"Opening serial port {port} @ {baud}")

        # Latest telemetry storage
        self._latest_telem = None
        self._telem_lock = threading.Lock()

        # Start serial reader
        self.serial_thread = AutonomySerialInterface(
            port,
            baud,
            self.on_new_telemetry
        )
        self.serial_thread.start()

        # Create publisher
        self.telem_publisher = self.create_publisher(Telemetry, 'telem', 10)

    # -------------------------------------------------
    def on_new_telemetry(self, telem):
        """
        Called every time a new telemetry packet arrives.
        This is where publishing will eventually happen.
        """
        with self._telem_lock:
            self._latest_telem = telem

        # Create telem message
        msg = Telemetry()
        
        # --- Populate header ---
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"  # or any relevant frame

        # --- Map telemetry tuple to message fields ---
        msg.counter         = telem[0]
        msg.state           = telem[1]

        msg.rc_throttle     = telem[2]
        msg.rc_steering     = telem[3]
        msg.rc_switch_a     = telem[4]
        msg.rc_switch_b     = telem[5]
        msg.rc_switch_c     = telem[6]
        msg.rc_switch_d     = telem[7]

        msg.ac_steering     = telem[8]
        msg.ac_throttle     = telem[9]

        msg.up_rssi         = telem[10]
        msg.up_lqi          = telem[11]
        msg.down_rssi       = telem[12]
        msg.down_lqi        = telem[13]

        msg.esc_voltage_raw = telem[14]
        msg.esc_current_raw = telem[15]
        msg.esc_rpm_raw     = telem[16]
        msg.esc_temp_raw    = telem[17]

        self.telem_publisher.publish(msg)

    # -------------------------------------------------
    def destroy_node(self):
        self.get_logger().info("Shutting down serial thread")
        self.serial_thread.stop()
        self.serial_thread.join()
        super().destroy_node()

# -------------------------------------------
def main():
    rclpy.init()
    node = AutonomyTelemetryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy.shutdown()

if __name__ == '__main__':
    main()
