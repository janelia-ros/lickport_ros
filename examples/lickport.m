% create matlab node to communicate with other nodes
matlab_node = ros2node("/matlab_node");

% create publisher to send position and velocity commands to the lickport
lickport_publisher = ros2publisher(matlab_node, ...
    "/lickport_joint_target", "sensor_msgs/JointState");

% create message to publish
lickport_message = ros2message("sensor_msgs/JointState");

% specify motor axis names
lickport_message.name = {'x', 'y', 'z'};

% first lickport position
lickport_message.position = [1000, 1000, 1000];
send(lickport_publisher, lickport_message);

%second lickport position
lickport_message.position = [100, 200, 300];
send(lickport_publisher, lickport_message);

% cleanup and close nodes when finished
clear lickport_publisher
clear lickport_message
clear matlab_node