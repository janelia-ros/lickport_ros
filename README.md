Lickport ROS Interface
======================

This is the ROS lickport interface.

Published Topics
----------------
* `/motor_duty_cycleXX` (`std_msgs/Float64`) - Get the motor duty cycle.  One topic is created for each motor attached.

Subscribed Topics
-----------------
* `/set_motor_duty_cycleXX` (`std_msgs/Float64`) - Set the motor duty cycle.  One topic is created for each motor attached.

Parameters
----------
* `serial` (int) - The serial number of the phidgets motor to connect to.  If -1 (the default), connects to any motor phidget that can be found.
