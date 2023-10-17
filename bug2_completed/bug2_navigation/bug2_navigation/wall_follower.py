
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from std_srvs.srv import *
import math

class WallFollowerClass(Node):
    def __init__(self):
        #Initialze a ros node
        super().__init__('WallFollowerController')

        #Create a subscriber to the /scan topic with the callback function self.clbk_laser
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.clbk_laser, 10)

        #Create a publisher to the /cmd_vel topic assigned to a variable called self.vel_pub
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        #Create a Service server with the Name "wall_follower_switch" using SetBool as a message structure and self.wall_follower_switch as the handeling function
        self.wall_follower_srv = self.create_service(SetBool, 'wall_follower_switch', self.wall_follower_callback)

        self.active = False

        self.regions = {
            'right': 0.0,
            'fright': 0.0,
            'front': 0.0,
            'fleft': 0.0,
            'left': 0.0,
        }
        self.state = 0
        self.state_dict = {
            0: 'find the wall',
            1: 'turn left',
            2: 'follow the wall',
        }

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def clbk_laser(self, msg):
        self.regions = {
            # 'right':  min(min(msg.ranges[180:299]), 1.0),
            # 'fright': min(min(msg.ranges[300:339]), 1.0),
            # 'front':  min(min(min(msg.ranges[0:19]), min(msg.ranges[340:359])),1.0),
            # 'fleft':  min(min(msg.ranges[20:59]), 1.0),
            # 'left':   min(min(msg.ranges[60:179]), 1.0),
            'right':  min(min(msg.ranges[180:299]), 1.0),
            'fright': min(min(msg.ranges[320:339]), 1.0),
            'front':  min(min(min(msg.ranges[0:9]), min(msg.ranges[350:359])),1.0),
            'fleft':  min(min(msg.ranges[20:39]), 1.0),
            'left':   min(min(msg.ranges[60:179]), 1.0),
        }
        print(self.regions)
        self.take_action()

        

    #Create the handeling function wall_follower_switch which should assign the transmitted boolean value to self.active
    def wall_follower_callback(self, req, res):
        self.active = req.data
        #res = SetBoolResponse()
        res.success = True
        res.message = 'Done!'
        return res

    def change_state(self, state):
        if state is not self.state:
            # rospy.loginfo('Wall follower - ['+str(state)+'] - '+str(self.state_dict[state]))
            self.get_logger().info('Wall follower - ['+str(state)+'] - '+str(self.state_dict[state]))
            self.state = state

    def take_action(self):
        regions = self.regions

        d = 0.9

        if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
            state_description = 'case 1 - nothing'
            self.change_state(0)
        elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
            state_description = 'case 2 - front'
            self.change_state(1)
        elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
            state_description = 'case 3 - fright'
            self.change_state(2)
        elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
            state_description = 'case 4 - fleft'
            self.change_state(0)
        elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
            state_description = 'case 5 - front and fright'
            self.change_state(1)
        elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
            state_description = 'case 6 - front and fleft'
            self.change_state(1)
        elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
            state_description = 'case 7 - front and fleft and fright'
            self.change_state(1)
        elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
            state_description = 'case 8 - fleft and fright'
            self.change_state(0)
        else:
            state_description = 'unknown case'
            self.get_logger().info(regions)

    def find_wall(self):
        msg = Twist()
        msg.linear.x = 0.3
        msg.angular.z = -0.5
        return msg

    def turn_left(self):
        msg = Twist()
        msg.angular.z = 0.5
        return msg

    def follow_the_wall(self):
        msg = Twist()
        msg.linear.x = 0.5
        return msg

    def timer_callback(self):
        if not self.active:
            return
        msg = Twist()
        if self.state == 0:
            msg = self.find_wall()
        elif self.state == 1:
            msg = self.turn_left()
        elif self.state == 2:
            msg = self.follow_the_wall()
            pass
        else:
            # rospy.logerr('Unknown state!')
            self.get_logger().error('Unknown state!')

        self.vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    controller = WallFollowerClass()

    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
