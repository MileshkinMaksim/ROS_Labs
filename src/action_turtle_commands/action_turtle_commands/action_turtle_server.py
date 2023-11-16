import time
import rclpy
import numpy as np
import math

from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from message_turtle_commands.action import MessageTurtleCommands


current_turtle_position = Pose()
starting_turtle_position = Pose()

class PositionCapturer(Node):
    def __init__(self):
        super().__init__("turtle1_pose_capture")
        self.subscriber_ = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.turtle1_pose_callback,
            10
        )

    def turtle1_pose_callback(self, msg):
        global current_turtle_position, starting_turtle_position
        current_turtle_position = msg
		#node for tracking the position of the turtle

class ActionTurtleServer(Node):
    def __init__(self):
        super().__init__('action_turtle_server')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self._action_server = ActionServer(
            self,
            action_type=MessageTurtleCommands,
            action_name='MessageTurtleCommands',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback
        )

        self.goal_position = Pose()

    def calculate_goal_point(self, s):
        global current_turtle_position
        self.goal_position.x += s * np.cos(current_turtle_position.theta)
        self.goal_position.y += s * np.sin(current_turtle_position.theta)
        self.goal_position.theta = current_turtle_position.theta
		#calculating the goal point based on speed and current angle

    def execute_command(self, request):
        command = request.command

        s = float(request.s)
        theta = math.radians(request.angle)
        msg = Twist()

        if command == "forward":
            msg.linear.x = s
            self.calculate_goal_point(s)
        elif command == "backward":
            msg.linear.x = -s
            self.calculate_goal_point(-s)
        elif command == "turn_left":
            msg.angular.z = theta
            self.goal_position.theta += theta
        elif command == "turn_right":
            msg.angular.z = -theta
            self.goal_position.theta -= theta
        return msg

    def positive_angle(self, angle):
        if angle < 0:
            return 2 * np.pi + angle
        return angle
		#converting negative angles to positive ones for processing

    def check_the_goal_reach(self):
        global current_turtle_position
        return (np.abs(current_turtle_position.x - self.goal_position.x) < 0.1 and np.abs(current_turtle_position.y - self.goal_position.y) < 0.1 and np.abs(self.positive_angle(current_turtle_position.theta) - self.positive_angle(self.goal_position.theta)) < 0.05)
		#checking for reaching the goal

    def execute_callback(self, goal_handle):
        global current_turtle_position, starting_turtle_position
        self.get_logger().info('Executing goal...')

        while current_turtle_position is None:  # wait for subscriber to receive at least one Pose()
            pass

        starting_turtle_position = Pose(x = current_turtle_position.x, y = current_turtle_position.y, theta = current_turtle_position.theta)

        self.goal_position = Pose(x = current_turtle_position.x, y = current_turtle_position.y, theta = current_turtle_position.theta)

        feedback_msg = MessageTurtleCommands.Feedback()

        result = MessageTurtleCommands.Result()

        turtle_movement_message = self.execute_command(goal_handle.request)
        self.publisher_.publish(turtle_movement_message)

        while not self.check_the_goal_reach():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                result.result = False
                return result
            current_odometry = np.sqrt((current_turtle_position.x - starting_turtle_position.x) ** 2 + (current_turtle_position.y - starting_turtle_position.y) ** 2)
            feedback_msg.odom = int(math.ceil(current_odometry))

            goal_handle.publish_feedback(feedback_msg)

            print(f"Curreunt Position: {current_turtle_position.x} {current_turtle_position.y} {current_turtle_position.theta}")
            print()
            print(f"Goal Position: {self.goal_position.x} {self.goal_position.y} {self.goal_position.theta}")
            print()
            print(f"Starting Position: {starting_turtle_position.x} {starting_turtle_position.y} {starting_turtle_position.theta}")
            print()


            time.sleep(0.5)

        goal_handle.succeed()

        starting_turtle_position = current_turtle_position

        result.result = True
        return result

    def cancel_callback(self, goal_handle):
		#Accept or reject a client request to cancel an action.
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT


def main(args=None):
    rclpy.init(args=args)

    action_turtle_server = ActionTurtleServer()

    pos_capturer = PositionCapturer()

		#launching multiple node execution to launch pos_capturer and server simultaniously
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(action_turtle_server)
    executor.add_node(pos_capturer)

		# Spin the nodes to execute the callbacks
    executor.spin()

		# Shutdown the nodes
    executor.shutdown()
    action_turtle_server.destroy_node()
    pos_capturer.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
