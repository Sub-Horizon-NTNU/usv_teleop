#!/usr/bin/env python3
"""Light + relay DRIVER (boat side) — runs on the boat, next to the Raspberry Pi.

Subscribes to `selene/light_state` (std_msgs/UInt8, published by
light_state_publisher on the controller/PC side) and sends the 1-byte UDP packet
(b"SL" + byte) to the Pi at 20 Hz. Fails safe (relay open, red on) if it stops
receiving the topic — e.g. the radio link to the controller drops.

This is the boat half of the split: the Pi/UDP driving stays local to the boat
so the lights/relay are robust to the control link, while the state itself is
computed with the controller on the PC (light_state_publisher.py).

Params: pi_ip, udp_port, timeout_s.
"""
import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8

MAGIC = b"SL"
FAILSAFE = 0b0010   # red on, relay open, amber/green off


class LightRelayDriver(Node):
    def __init__(self):
        super().__init__("light_relay_driver")
        self.pi_ip = self.declare_parameter("pi_ip", "192.168.2.5").value
        self.port = int(self.declare_parameter("udp_port", 5005).value)
        self.timeout = float(self.declare_parameter("timeout_s", 0.5).value)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._byte = FAILSAFE
        self._last_rx = self.get_clock().now()
        self._stale = False

        self.create_subscription(UInt8, "selene/light_state", self._on_state, 10)
        self.create_timer(0.05, self._tick)     # 20 Hz send
        self.get_logger().info(
            f"light_relay_driver: selene/light_state -> {self.pi_ip}:{self.port}")

    def _on_state(self, msg):
        self._byte = msg.data
        self._last_rx = self.get_clock().now()

    def _tick(self):
        age = (self.get_clock().now() - self._last_rx).nanoseconds * 1e-9
        stale = age > self.timeout
        if stale != self._stale:
            self.get_logger().warn("selene/light_state stale -> failsafe (red, relay open)"
                                   if stale else "selene/light_state OK")
            self._stale = stale
        b = FAILSAFE if stale else (self._byte & 0xFF)
        self.sock.sendto(MAGIC + bytes([b]), (self.pi_ip, self.port))


def main():
    rclpy.init()
    try:
        rclpy.spin(LightRelayDriver())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
