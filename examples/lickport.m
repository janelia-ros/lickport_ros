matlab_node = ros2node("/matlab_node");
lickport_publisher = ros2publisher(matlab_node,"/lickport_joint_target", "sensor_msgs/JointState");
lickport_message = ros2message("sensor_msgs/JointState");
lickport_message.name = {'x', 'y', 'z'};

lickport_message.position = [1000, 1000, 1000];
send(lickport_publisher,lickport_message);

lickport_message.position = [100, 200, 300];
send(lickport_publisher,lickport_message);

clear lickport_publisher
clear lickport_message
clear matlab_node