# usv_teleop

ROS 2 Python package for **joystick manual override of Selene** and for forwarding boat state to indicator lights and a relay. It is used with [usv_controller](https://github.com/Sub-Horizon-NTNU/usv_controller), which sends the actual PX4 actuator commands.

## Components

- [joystick_teleop.py](usv_teleop/joystick_teleop.py): converts `sensor_msgs/Joy` to `selene/manual/cmd` (`geometry_msgs/Twist`), `selene/manual/override`, `selene/manual/estop` and `selene/arm` (`std_msgs/Bool`).
- [light_state_sender.py](usv_teleop/light_state_sender.py): reads PX4 status and manual-control state and sends light/relay state directly over UDP.
- [light_state_publisher.py](usv_teleop/light_state_publisher.py) and [light_relay_driver.py](usv_teleop/light_relay_driver.py): split the state calculation and boat-side UDP forwarding across two machines. The relay driver uses a 0.5 s message timeout by default.
- [launch/](launch/): joystick and relay-driver launch files.

The default joystick mapping is Start to toggle the arm request, hold LB for manual override, and B for an emergency-stop/disarm request. Releasing LB hands control back to the controller's guidance logic; it is not a dedicated stop command. These are software requests, and their effect depends on the receiving controller.

## Build and integration

Place this package in a ROS 2 workspace's `src/` directory, with the matching `px4_msgs` definitions available for the light-state nodes. The joystick launch also needs the ROS 2 `joy` package. After sourcing ROS 2, build from the workspace root:

```sh
colcon build --packages-up-to usv_teleop
source install/setup.bash
```

The joystick entry point is `ros2 launch usv_teleop usv_teleop.launch.py`; axis/button indices and output scales are declared in the joystick node. Review the mapping before connecting to a live controller.

UDP light messages use `SL` followed by one state byte, on port 5005 by default. The matching Pi receiver is documented in [usv_bringup/pi_lights](https://github.com/Sub-Horizon-NTNU/usv_bringup/tree/master/pi_lights).
