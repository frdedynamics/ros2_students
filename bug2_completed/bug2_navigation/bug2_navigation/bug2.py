#! /usr/bin/env python

# import rospy
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
import math
import time
# import actionlib
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
# from tf import transformations
from tf_transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import *
from dat160_interfaces.srv import GoToPoint
from dat160_interfaces.action import Bug2


class bug2(Node):
    def __init__(self):
        #Initialze a ros node
        super().__init__('Bug2Controller')

        #Create a subscriber to the /scan topic with the callback function self.clbk_laser
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.clbk_laser, 10)

        #Create a subscriber to the /odom topic with the callback function self.clbk_odom
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.clbk_odom, 10)

        self.srv_client_go_to_point_ = None
        self.srv_client_wall_follower_ = None
        self.yaw_ = 0
        self.yaw_error_allowed_ = 5 * (math.pi / 180) # 5 degrees
        self.dist_precision_ = 0.1

        self.position_ = Point()
        self.desired_position_ = Point()

        self.regions_ = None
        self.state_desc_ = ['Go to point', 'wall following', 'Goal reached']
        # 0 - go to point; 1 - wall following; 2 - Goal reached
        self.state_ = 0

        
        

        #create a service client connected to the 'go_to_point_switch' server and wait for it to be available
        self.srv_client_go_to_point = self.create_client(GoToPoint, 'go_to_point_switch')
        while not self.srv_client_go_to_point.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.go_to_point_request = GoToPoint.Request()

        #create a service client connected to the 'wall_follower_switch' server and wait for it to be available
        self.srv_client_wall_follower = self.create_client(SetBool, 'wall_follower_switch')
        while not self.srv_client_wall_follower.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.wall_follower_request = SetBool.Request()

        #create an action server with a custom message defined in the assignment description. As an execute_cb use the function self.bug2_execute_cb
        self.action_server = ActionServer(self, Bug2, 'bug2',self.bug2_execute_cb)

        self.result = Bug2.Result()
        self.feedback = Bug2.Feedback()


    def clbk_odom(self, msg):
        self.position_ = msg.pose.pose.position

        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)

        euler = euler_from_quaternion(quaternion)
        self.yaw_ = euler[2]

    def clbk_laser(self, msg):
        self.regions_ = {
            'right':  min(min(msg.ranges[180:299]), 1.0),
            'fright': min(min(msg.ranges[300:339]), 1.0),
            'front':  min(min(min(msg.ranges[0:19]), min(msg.ranges[340:359])),1.0),
            'fleft':  min(min(msg.ranges[20:59]), 1.0),
            'left':   min(min(msg.ranges[60:179]), 1.0),
        }   
        # self.get_logger().info(str(self.regions_)) 

    def change_state(self, state):
        self.state_ = state
        log = "state changed: %s" % self.state_desc_[state]
        self.get_logger().info(log)

        if self.state_ == 0:
            self.go_to_point_request.move_switch = True
            self.wall_follower_request.data = False
        elif self.state_ == 1:
            self.last_line_position = self.position_
            self.go_to_point_request.move_switch = False
            self.wall_follower_request.data = True
        elif self.state_ == 2:
            self.go_to_point_request.move_switch = False
            self.wall_follower_request.data = False

        self.go_to_point_request.target_position = self.desired_position_
        # self.get_logger().info("Request sent: "+ str(self.go_to_point_request))
        self.go_to_point_future = self.srv_client_go_to_point.call_async(self.go_to_point_request)
        self.wall_follower_future = self.srv_client_wall_follower.call_async(self.wall_follower_request)

    def distance_to_line(self, p0):
        # p0 is the current position
        # p1 and p2 points define the line
        p1 = self.initial_position_
        p2 = self.desired_position_
        # here goes the equation
        up_eq = math.fabs((p2.y - p1.y) * p0.x - (p2.x - p1.x) * p0.y + (p2.x * p1.y) - (p2.y * p1.x))
        lo_eq = math.sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2))
        distance = up_eq / lo_eq

        return distance

    def distance_to_target(self, pos):
        distance = math.sqrt(pow(self.desired_position_.y - pos.y,2) + pow(self.desired_position_.x - pos.x, 2))
        return distance

    def normalize_angle(self, angle):
        if(math.fabs(angle) > math.pi):
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle

    def bug2_execute_cb(self, goal_handle):
        self.get_logger().info("starting bug2 algorythm")
        self.initial_position_ = self.position_
        self.desired_position_ = goal_handle.request.target_position
        # initialize going to the point
        self.change_state(0)

        while self.state_ != 2:
      
            rclpy.spin_once(self)
            if goal_handle.is_cancel_requested:
                self.aserver.set_preempted()
                goal_handle.canceled()
                self.get_logger().info("Client requested cancelling of the goal")
                return Bug2.Result()
           
            if self.regions_ == None:
                continue
           
            position = self.position_
            distance_position_to_line = self.distance_to_line(position)

            if self.distance_to_target(position) < self.dist_precision_:
               self.change_state(2)
               break

            if self.state_ == 0:
                desired_yaw = math.atan2(self.desired_position_.y - self.position_.y, self.desired_position_.x - self.position_.x)
                err_yaw = self.normalize_angle(desired_yaw - self.yaw_)
                if self.regions_['front'] < 0.9 and math.fabs(err_yaw) < 0.3:
                    self.change_state(1)


            elif self.state_ == 1:
                if distance_position_to_line < 0.1:
                    if (self.distance_to_target(position)+0.15) < self.distance_to_target(self.last_line_position):
                        self.change_state(0)

            #
            self.feedback.current_position = self.position_
            goal_handle.publish_feedback(self.feedback)

            time.sleep(0.01)

        self.result.base_position = self.position_
        return self.result


def main(args=None):
    rclpy.init(args=args)

    controller = bug2()

    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()