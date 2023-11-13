%% ELE306 turtlebot lab number 1
clc; 
clear; 
close all;

% Setting up environment
setenv("ROS_DOMAIN_ID","30");
% Initializing ros node
braitenberControllerNode = ros2node("/braitenberg_controller");

% Creating subscriber to laser scan and publisher to cmd velocity
pause(3)
laserSub = ros2subscriber(braitenberControllerNode,"/scan","sensor_msgs/LaserScan","Reliability","besteffort","Durability","volatile","Depth",5);
cmdvalPub = ros2publisher(braitenberControllerNode, "/cmd_vel", "geometry_msgs/Twist");
pause(3)

% Defining message for publisher
cmdvelMsg = ros2message(cmdvalPub);

% Defining variables

% Front left and front right distances
lidar_left_front = 0;
lidar_right_front = 0;

% Front left and front right angles
left_range = 40;
right_range = 320;

% Distance threshold
lidar_threshold = 0.35;

% For ever loop
while true
    % Reading out the scan data 
    [scanData,status,statustext] = receive(laserSub,10);
    lidar_left_front = scanData.ranges(left_range);
    lidar_right_front = scanData.ranges(right_range);

    % Plotting the scan data for fun :)
    angles = linspace(-pi,pi,360);
    scan = lidarScan(scanData.ranges, angles);
    plot(scan);
    
    % Velocity commands if no obstacle
    cmdvelMsg.linear.x = 0.1;
    cmdvelMsg.angular.z = 0.0;

    % Velocity commands if obstacles on both sides
    if lidar_right_front < lidar_threshold && lidar_left_front < lidar_threshold
        cmdvelMsg.linear.x = 0.0;
        cmdvelMsg.angular.z = 0.0;  
        disp("Stuck, ")
        disp(lidar_left_front) 
        disp(lidar_right_front)
       
    else
        % Velocity commands if obstacles on the right side -> turning left
        if lidar_right_front < lidar_threshold
            cmdvelMsg.angular.z = 0.45;
            disp("turning left");
        end 
        % Velocity commands if obstacles on the left side -> turning right
        if lidar_left_front < lidar_threshold
            cmdvelMsg.angular.z = -0.45;
            disp("turnting right");
        end
    end 
    % Send velocity commands to turtlebot
    send(cmdvalPub, cmdvelMsg)
end

