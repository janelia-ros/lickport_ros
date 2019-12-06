Lickport ROS Interface
======================

This is the ROS lickport interface.

Published Topics
----------------
* `/lickport_joint_state` (`sensor_msgs/JointState`) - A joint state message containing the current state of all joints.

Subscribed Topics
-----------------
* `/set_motor_duty_cycleXX` (`std_msgs/Float64`) - Set the motor duty cycle.  One topic is created for each motor attached.

Parameters
----------
* `serial` (int) - The serial number of the phidgets motor to connect to.  If -1 (the default), connects to any motor phidget that can be found.

Command Line Examples
---------------------

```bash
ros2 run lickport lickport
ros2 topic echo /joint_states
ros2 service call /set_current_limit00 phidgets_msgs/SetCurrentLimit "current_limit: 1.0"
```
