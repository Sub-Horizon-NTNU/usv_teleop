#!/usr/bin/env python3
"""Light-state sender (companion side): boat state -> UDP -> Raspberry Pi GPIO.

Reads PX4 arming + the manual topics, and UDP-sends a 1-byte state packet
(b"SL" + byte: bit0 relay, bit1 red, bit2 amber, bit3 green) to the Pi at 20 Hz.
The Pi (see pi_lights/lights.py) just sets GPIO from it. No PX4 PWM involved.

State -> lights (mutually exclusive):
    armed & !estop & !manual -> green (guided), relay closed
    armed & !estop &  manual -> amber (manual),  relay closed
    otherwise                -> red,             relay open

Params: pi_ip (Pi address), udp_port.
"""
import socket
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool
from px4_msgs.msg import VehicleStatus

MAGIC = b"SL"
ARMED = 2  # VehicleStatus.ARMING_STATE_ARMED


class LightStateSender(Node):
    def __init__(self):
        super().__init__("light_state_sender")
        self.pi_ip = self.declare_parameter("pi_ip", "192.168.2.5").value
        self.port = int(self.declare_parameter("udp_port", 5005).value)
        self.armed = self.manual = self.estop = False
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status",
                                 self._on_status, qos)
        self.create_subscription(Bool, "selene/manual/override",
                                 lambda m: setattr(self, "manual", m.data), 10)
        self.create_subscription(Bool, "selene/manual/estop",
                                 lambda m: setattr(self, "estop", m.data), 10)
        self.create_timer(0.05, self._tick)   # 20 Hz
        self.get_logger().info(f"light_state_sender -> {self.pi_ip}:{self.port}")

    def _on_status(self, msg):
        self.armed = (msg.arming_state == ARMED)

    def _tick(self):
        active = self.armed and not self.estop
        relay = active
        red = not active
        amber = active and self.manual
        green = active and not self.manual
        b = (int(relay) << 0) | (int(red) << 1) | (int(amber) << 2) | (int(green) << 3)
        self.sock.sendto(MAGIC + bytes([b]), (self.pi_ip, self.port))


def main():
    rclpy.init()
    try:
        rclpy.spin(LightStateSender())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
