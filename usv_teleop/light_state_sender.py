#!/usr/bin/env python3
"""Light-state sender (companion side): boat state -> UDP -> Raspberry Pi GPIO.

Reads PX4 arming + the manual topics and UDP-sends a 1-byte state packet
(b"SL" + byte: bit0 relay, bit1 red, bit2 amber, bit3 green) to the Pi at 20 Hz.
The Pi (pi_lights/lights.py) just sets GPIO from it. No PX4 PWM involved.

Armed is taken from TWO independent topics so a flaky one can't stick the lights
on red: vehicle_status.arming_state==2 OR vehicle_control_mode.flag_armed.
Both the raw arming_state and flag_armed are logged on change for debugging.

State -> lights (mutually exclusive):
    armed & !estop & !manual -> green (guided), relay closed
    armed & !estop &  manual -> amber (manual),  relay closed
    otherwise                -> red,             relay open

Params: pi_ip, udp_port.
"""
import socket
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool
from px4_msgs.msg import VehicleStatus, VehicleControlMode

MAGIC = b"SL"
ARMED = 2  # VehicleStatus.ARMING_STATE_ARMED (verified px4_msgs release/1.16)


class LightStateSender(Node):
    def __init__(self):
        super().__init__("light_state_sender")
        self.pi_ip = self.declare_parameter("pi_ip", "192.168.2.5").value
        self.port = int(self.declare_parameter("udp_port", 5005).value)

        self._vs_armed = False        # from vehicle_status.arming_state
        self._cm_armed = False        # from vehicle_control_mode.flag_armed
        self.manual = False
        self.estop = False

        self._last_arming_state = -1
        self._last_flag_armed = None
        self._last_b = -1
        self._status_seen = False

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status",
                                 self._on_status, qos)
        self.create_subscription(VehicleControlMode, "/fmu/out/vehicle_control_mode",
                                 self._on_control_mode, qos)
        self.create_subscription(Bool, "selene/manual/override",
                                 lambda m: setattr(self, "manual", m.data), 10)
        self.create_subscription(Bool, "selene/manual/estop",
                                 lambda m: setattr(self, "estop", m.data), 10)

        self.create_timer(0.05, self._tick)     # 20 Hz send
        self.create_timer(2.0, self._heartbeat)
        self.get_logger().info(f"light_state_sender -> {self.pi_ip}:{self.port}")

    def _on_status(self, msg):
        self._status_seen = True
        self._vs_armed = (msg.arming_state == ARMED)
        if msg.arming_state != self._last_arming_state:
            self.get_logger().info(f"vehicle_status.arming_state = {msg.arming_state} "
                                   f"(armed={self._vs_armed})")
            self._last_arming_state = msg.arming_state

    def _on_control_mode(self, msg):
        self._status_seen = True
        self._cm_armed = bool(msg.flag_armed)
        if msg.flag_armed != self._last_flag_armed:
            self.get_logger().info(f"vehicle_control_mode.flag_armed = {msg.flag_armed}")
            self._last_flag_armed = msg.flag_armed

    def _heartbeat(self):
        if not self._status_seen:
            self.get_logger().warn("no /fmu/out/vehicle_status or /vehicle_control_mode yet "
                                   "-> armed stays False (red). agent up? topics bridged? QoS?")

    def _tick(self):
        armed = self._vs_armed or self._cm_armed
        active = armed and not self.estop
        relay = active
        red = not active
        amber = active and self.manual
        green = active and not self.manual
        b = (int(relay) << 0) | (int(red) << 1) | (int(amber) << 2) | (int(green) << 3)
        if b != self._last_b:
            self.get_logger().info(
                f"armed={armed} (vs={self._vs_armed} cm={self._cm_armed}) "
                f"manual={self.manual} estop={self.estop} "
                f"-> relay={relay} red={red} amber={amber} green={green} (byte={b:04b})")
            self._last_b = b
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
