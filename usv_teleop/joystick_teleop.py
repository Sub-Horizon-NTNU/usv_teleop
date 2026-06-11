#!/usr/bin/env python3
"""Joystick teleop for the Selene USV manual-override path.

Subscribes to sensor_msgs/Joy and republishes, on the topics that usv_controller
consumes for manual override:

  selene/manual/override  std_msgs/Bool   True while the deadman button is held
  selene/manual/cmd       geometry_msgs/Twist  body-frame surge/sway + yaw rate
  selene/manual/estop     std_msgs/Bool   True while the e-stop button is held

The controller stays the single PX4 offboard publisher; this node never touches
px4_msgs. Axis/button indices and scales are parameters (defaults suit a common
Xbox-style pad). Override is a held deadman so releasing the pad hands control
back to LOS guidance.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class JoystickTeleop(Node):

    def __init__(self):
        super().__init__('joystick_teleop')

        # Axis / button mapping (Xbox-style defaults).
        self.declare_parameter('surge_axis', 1)        # left stick vertical -> forward
        self.declare_parameter('sway_axis', 0)         # left stick horizontal -> lateral
        self.declare_parameter('yaw_axis', 3)          # right stick horizontal -> yaw rate
        self.declare_parameter('deadman_button', 4)    # LB: hold to take manual control
        self.declare_parameter('estop_button', 1)      # B: e-stop / disarm
        self.declare_parameter('arm_button', 7)        # Start: press to toggle arm/disarm

        # Output scaling.
        self.declare_parameter('max_surge', 2.0)       # m/s
        self.declare_parameter('max_sway', 1.0)        # m/s
        self.declare_parameter('max_yaw_rate', 1.0)    # rad/s
        self.declare_parameter('deadzone', 0.05)

        self.surge_axis = self.get_parameter('surge_axis').value
        self.sway_axis = self.get_parameter('sway_axis').value
        self.yaw_axis = self.get_parameter('yaw_axis').value
        self.deadman_button = self.get_parameter('deadman_button').value
        self.estop_button = self.get_parameter('estop_button').value
        self.arm_button = self.get_parameter('arm_button').value
        self.max_surge = self.get_parameter('max_surge').value
        self.max_sway = self.get_parameter('max_sway').value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').value
        self.deadzone = self.get_parameter('deadzone').value

        self.override_pub = self.create_publisher(Bool, 'selene/manual/override', 10)
        self.cmd_pub = self.create_publisher(Twist, 'selene/manual/cmd', 10)
        self.estop_pub = self.create_publisher(Bool, 'selene/manual/estop', 10)
        self.arm_pub = self.create_publisher(Bool, 'selene/arm', 10)

        self.armed = False          # latched arm state, toggled by arm_button
        self._prev_arm_btn = False

        self.create_subscription(Joy, 'joy', self.joy_cb, 10)
        self.get_logger().info('Joystick teleop started (Start=arm/disarm, hold LB=manual)')

    def _axis(self, axes, idx):
        if idx < 0 or idx >= len(axes):
            return 0.0
        v = axes[idx]
        return 0.0 if abs(v) < self.deadzone else v

    def _button(self, buttons, idx):
        return idx >= 0 and idx < len(buttons) and buttons[idx] == 1

    def joy_cb(self, msg: Joy):
        override = self._button(msg.buttons, self.deadman_button)
        estop = self._button(msg.buttons, self.estop_button)

        # Arm: toggle on the rising edge of the arm button. E-stop forces disarm.
        arm_btn = self._button(msg.buttons, self.arm_button)
        if arm_btn and not self._prev_arm_btn:
            self.armed = not self.armed
            self.get_logger().info(f"{'ARM' if self.armed else 'DISARM'} requested")
        self._prev_arm_btn = arm_btn
        if estop:
            self.armed = False

        self.override_pub.publish(Bool(data=override))
        self.estop_pub.publish(Bool(data=estop))
        self.arm_pub.publish(Bool(data=self.armed))

        cmd = Twist()
        if override:
            cmd.linear.x = self._axis(msg.axes, self.surge_axis) * self.max_surge
            cmd.linear.y = self._axis(msg.axes, self.sway_axis) * self.max_sway
            cmd.angular.z = self._axis(msg.axes, self.yaw_axis) * self.max_yaw_rate
        # else: zeros, but the controller ignores cmd unless override is active.
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
