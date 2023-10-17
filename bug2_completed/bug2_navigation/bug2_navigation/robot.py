import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point
from dat160_interfaces.action import Bug2

class RobotClass(Node):
    def __init__(self):
        #Initialze a ros node
        super().__init__('RobotClass')

        #Create an action client connected to the "bug2_nav_action" action server
        self.bug2_action_client = ActionClient(self, Bug2, 'bug2')

        #Wait for the action server to be active
        self.bug2_action_client.wait_for_server()

    def feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info("Current position of the robot: "+str(feedback.current_position))

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self.bug2_get_result_future = goal_handle.get_result_async()
        self.bug2_get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("The robot finished at position: "+str(result.base_position))
        rclpy.shutdown()

    def run(self):
        #create an action goal with target_position being a Point with x=0 and y=8
        self.target_position = Point()
        self.target_position.x = 0.0
        self.target_position.y = 8.0

        goal_msg = Bug2.Goal()
        goal_msg.target_position = self.target_position

        #send the goal to the action server using the existing self.done_cb, self.active_cb and self.feedback_cb functions
        self.bug2_send_goal_future = self.bug2_action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_cb)

        self.bug2_send_goal_future.add_done_callback(self.goal_response_callback)

        return


def main(args=None):
    rclpy.init(args=args)

    robot = RobotClass()

    robot.run()

    rclpy.spin(robot)


if __name__ == '__main__':
    main()

