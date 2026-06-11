#!/usr/bin/env python3
"""Light-state PUBLISHER (controller side) — runs where the controller + arm/manual
state is (the PC).

Computes the relay + light state from PX4 arming and the manual topics and
publishes it as std_msgs/UInt8 on `selene/light_state`
(bit0 relay, bit1 red, bit2 amber, bit3 green). The boat-side
`light_relay_driver` turns that topic into UDP for the Pi.

This split keeps the state logic with the controller (PC) and the Pi/UDP driving
on the boat — see light_relay_driver.py.

Armed is taken from two independent topics so a flaky one can't stick it on red:
vehicle_status.arming_state==2 OR vehicle_control_mode.flag_armed.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, UInt8
from px4_msgs.msg import VehicleStatus, VehicleControlMode

ARMED = 2  # VehicleStatus.ARMING_STATE_ARMED


class LightStatePublisher(Node):
    def __init__(self):
        super().__init__("light_state_publisher")
        self._vs_armed = False
        self._cm_armed = False
        self.manual = False
        self.estop = False
        self._last_b = -1
        self._last_arming_state = -1
        self._last_flag_armed = None
        self._seen = False

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status",
                                 self._on_status, qos)
        self.create_subscription(VehicleControlMode, "/fmu/out/vehicle_control_mode",
                                 self._on_cm, qos)
        self.create_subscription(Bool, "selene/manual/override",
                                 lambda m: setattr(self, "manual", m.data), 10)
        self.create_subscription(Bool, "selene/manual/estop",
                                 lambda m: setattr(self, "estop", m.data), 10)
        self.pub = self.create_publisher(UInt8, "selene/light_state", 10)
        self.create_timer(0.05, self._tick)     # 20 Hz
        self.create_timer(2.0, self._heartbeat)
        self.get_logger().info("light_state_publisher -> selene/light_state")

    def _on_status(self, msg):
        self._seen = True
        self._vs_armed = (msg.arming_state == ARMED)
        if msg.arming_state != self._last_arming_state:
            self.get_logger().info(f"vehicle_status.arming_state = {msg.arming_state} "
                                   f"(armed={self._vs_armed})")
            self._last_arming_state = msg.arming_state

    def _on_cm(self, msg):
        self._seen = True
        self._cm_armed = bool(msg.flag_armed)
        if msg.flag_armed != self._last_flag_armed:
            self.get_logger().info(f"vehicle_control_mode.flag_armed = {msg.flag_armed}")
            self._last_flag_armed = msg.flag_armed

    def _heartbeat(self):
        if not self._seen:
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
                f"manual={self.manual} estop={self.estop} -> byte={b:04b}")
            self._last_b = b
        self.pub.publish(UInt8(data=b))


def main():
    rclpy.init()
    try:
        rclpy.spin(LightStatePublisher())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
