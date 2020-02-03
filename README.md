Lickport ROS Interface
======================

This is the ROS lickport interface.

Published Topics
----------------
* `/lickport_joint_state` (`sensor_msgs/JointState`) - A joint state message containing the current state of all joints.

Subscribed Topics
-----------------
* `/lickport_joint_target` (`joint_control_msgs/JointTarget`) - Set joint state targets.

Parameters
----------
* `serial` (int) - The serial number of the phidgets motor to connect to.  If -1 (the default), connects to any motor phidget that can be found.

Command Line Examples
---------------------

```bash
ros2 run lickport lickport
ros2 topic echo /lickport_joint_state
ros2 topic pub -1 /lickport_joint_target joint_control_msgs/JointTarget "{name: [x,y,z], position: [1000,1000,1000]}"
```
