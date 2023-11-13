%% ELE306 turtlebot lab number 2
clc; 
clear; 
close all;
import ETS3.*

% Setting up environment
setenv("ROS_DOMAIN_ID","30");
% Initializing ros node
armControllerNode = ros2node("/open_manipulator_controller");

trajPub = ros2publisher(armControllerNode, "/arm_controller/joint_trajectory", "trajectory_msgs/JointTrajectory");
pause(3)

% Defining message for publisher
trajMsg = ros2message(trajPub);
trajMsg.joint_names = {'joint1', 'joint2', 'joint3', 'joint4'};

%%
% Defining the robotic arm
L1 = 0.077;
L2 = 0.128;
L3 = 0.024;
L4 = 0.124;
L5 = 0.126;
L6 = sqrt(L2*L2 + L3*L3);
beta = atan(L3/L2);


j1 = Revolute('d', L1, 'a', 0, 'alpha', pi/2, 'offset', pi);
j2 = Revolute('d', 0, 'a', L6, 'alpha', 0, 'offset', beta + pi/2);
j3 = Revolute('d', 0, 'a', L4, 'alpha', 0, 'offset', -beta + pi/2);
j4 = Revolute('d', 0, 'a', L5, 'alpha', 0, 'offset', 0);

robot = SerialLink([j1 j2 j3 j4],'name', 'my robot');
robot.qlim = [-3.14, +3.14; -1.57, +1.57; -1.40, +1.57; -1.57, 1.57];


robot.plot([0, 0, 0, 0])

%% 
% Make sure the gripper is open  /gripper_controller/gripper_cmd

% [client,goalMsg] = ros2actionclient(armControllerNode,"/gripper_controller/gripper_cmd","control_msgs/GripperCommand");
% 
% status = waitForServer(client)
% goalMsg.command.position= 0.0;
% goalMsg.command.max_effort = 0.0;
% 
% 
% %callbackOpts = ros2ActionSendGoalOptions(FeedbackFcn=@helperFeedbackCallback,ResultFcn=@helperResultCallback);
% goalHandle = sendGoal(client,goalMsg); %, callbackOpts);
% exStatus = getStatus(client,goalHandle)
% 
% resultMsg = getResult(goalHandle);
% rosShowDetails(resultMsg)

%% 
% Gripper subscriber
% Make sure the gripper is open

gripperPub = ros2publisher(armControllerNode, "/gripper_topic", "std_msgs/Float64");
gripperMsg = ros2message(gripperPub);
gripperMsg.data = 0.0;
send(gripperPub,gripperMsg)


%%
% Go first to "zero" point 
point1 = ros2message('trajectory_msgs/JointTrajectoryPoint');
point1.positions =  [0.0, 0.0, 0, 0.0];
point1.velocities = [0, 0, 0,0];
point1.accelerations = [0, 0, 0, 0];
point1.effort = [0, 0, 0, 0];
point1.time_from_start.sec = int32(2);

% Check in the plot that it looks okay

robot.plot(point1.positions);

%%

T_robot_goal_1 = SE3(0.25, 0 , -0.01) * SE3.rpy(0,0,90, 'deg');
q1 = robot.ikcon(T_robot_goal_1, point1.positions) % this takes into account joint limits and the inital position

% Check in the plot that it looks okay

robot.plot(q1)

%%
point2 = ros2message('trajectory_msgs/JointTrajectoryPoint');
point2.positions =  q1;
point2.velocities = [0, 0, 0,0];
point2.accelerations = [0, 0, 0, 0];
point2.effort = [0, 0, 0, 0];
point2.time_from_start.sec = int32(5);
trajMsg.points = [point1, point2];
send(trajPub, trajMsg)

%% 
% Grip the cup  /gripper_topic
gripperMsg.data = 0.016;
send(gripperPub,gripperMsg)

%% 
% Take the cup "in"

T_robot_goal_2 = SE3(0.10, 0 , 0.15) * SE3.rpy(0,0,0, 'deg');
q2 = robot.ikcon(T_robot_goal_2, point2.positions) % this takes into account joint limits and the inital position

% Check in the plot that it looks okay

robot.plot(q2);

%%
point3 = ros2message('trajectory_msgs/JointTrajectoryPoint');
point3.positions =  q2;
point3.velocities = [0, 0, 0,0];
point3.accelerations = [0, 0, 0, 0];
point3.effort = [0, 0, 0, 0];
point3.time_from_start.sec = int32(5);
trajMsg.points = point3;
send(trajPub, trajMsg);
