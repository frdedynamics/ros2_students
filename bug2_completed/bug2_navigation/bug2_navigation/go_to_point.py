
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from std_srvs.srv import *
from dat160_interfaces.srv import GoToPoint
import math

class GoToPointClass(Node):
    def __init__(self):
        #Initialze a ros node
        super().__init__('GoToPointController')

        #Create a subscriber to the /odom topic with the callback function self.clbk_odom
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.clbk_odom, 10)

        #Create a publisher to the /cmd_vel topic assigned to a variable called self.vel_pub
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        #Create a Service server with the Name "go_to_point_switch" using a custom message structure given in the assignment description and self.go_to_point_switch as the handeling function
        self.go_to_point_srv = self.create_service(GoToPoint, 'go_to_point_switch', self.go_to_point_callback)

        self.position = Point()
        self.yaw = 0.0
        self.state = 0
        self.desired_position = Point()
        # self.desired_position.x = 2.0
        self.yaw_precision = math.pi / 90
        self.dist_precision = 0.05
        self.active = False

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    #Create the handeling function go_to_point_switch which should assign the transmitted boolean value to self.active and the transmitted Point value to self.desired_position
    def go_to_point_callback(self, req, res):
        self.get_logger().info("Recieved: "+str(req.move_switch))
        self.active = req.move_switch
        self.desired_position = req.target_position
        res.success = True
        return res

    def clbk_odom(self, msg):
        self.position = msg.pose.pose.position

        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]

    def change_state(self, state):
        self.state = state
        self.get_logger().info('State changed to '+str(self.state))

    def normalize_angle(self, angle):
        if(math.fabs(angle) > math.pi):
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle

    def done(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.vel_pub.publish(twist_msg)
        self.get_logger().info('go-to-point -> finished')

    def fix_yaw(self, des_pos):
        desired_yaw = math.atan2(des_pos.y - self.position.y, des_pos.x - self.position.x)
        err_yaw = self.normalize_angle(desired_yaw - self.yaw)

        twist_msg = Twist()
        if math.fabs(err_yaw) > self.yaw_precision:
            twist_msg.angular.z = 0.3 if err_yaw > 0.0 else -0.3

        self.vel_pub.publish(twist_msg)
        self.get_logger().info('Yaw error: ['+str(err_yaw)+']')
        # state change conditions
        if math.fabs(err_yaw) <= self.yaw_precision:
            self.change_state(1)

    def go_straight_ahead(self, des_pos):
        desired_yaw = math.atan2(des_pos.y - self.position.y, des_pos.x - self.position.x)
        err_yaw = self.normalize_angle(desired_yaw - self.yaw)
        err_pos = math.sqrt(pow(des_pos.y - self.position.y, 2) + pow(des_pos.x - self.position.x, 2))

        if err_pos > self.dist_precision:
            twist_msg = Twist()
            twist_msg.linear.x = 0.3
            self.vel_pub.publish(twist_msg)
        else:
            self.get_logger().info('Position error: '+str(err_pos))
            self.change_state(2)

        # state change conditions
        if math.fabs(err_yaw) > self.yaw_precision:
            self.change_state(0)

    def timer_callback(self):
        if not self.active:
            dist_to_target = math.sqrt(pow(self.desired_position.y - self.position.y, 2) + pow(self.desired_position.x - self.position.x, 2))
            if dist_to_target < 0.15:
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                self.vel_pub.publish(twist_msg)
            return
        # self.get_logger().info("Go to Point active status: "+str(self.active))
        if self.state == 0:
            self.fix_yaw(self.desired_position)
        elif self.state == 1:
            self.go_straight_ahead(self.desired_position)
        elif self.state == 2:
            self.done()
            self.active = False
            
        else:
            # rospy.logerr('Unknown state!')
            self.get_logger().error('Unknown state!')

def main(args=None):
    rclpy.init(args=args)

    controller = GoToPointClass()

    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
