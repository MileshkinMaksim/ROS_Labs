import sys
import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node
from message_turtle_commands.action import MessageTurtleCommands


class ActionTurtleClient(Node):

    def __init__(self, commands):
        super().__init__('action_turtle_client')
        self.command_params = {
            'forward': [1, 0],
            'backward': [1, 0],
            'turn_left': [0, 90],
            'turn_right': [0, 90]
        }

        self.commands = commands

        if commands == []:
            self.get_logger().info("No commands recieved")
            raise Exception()
        else:
            for command in self.commands:
                if command not in self.command_params.keys():
                    self.get_logger().info(f"WRONG Command {command} .\nAvailable commands here: {self.command_params.keys()}")
                    raise Exception()

        self.command_number = 0

        self._action_client = ActionClient(
		self,
		MessageTurtleCommands,
		'MessageTurtleCommands'
		)

    def send_goal(self):
        goal_msg = MessageTurtleCommands.Goal()

        goal_msg.command = self.commands[self.command_number]
        goal_msg.s = self.command_params[goal_msg.command][0]
        goal_msg.angle = self.command_params[goal_msg.command][1]

        while not self._action_client.wait_for_server():
            self.get_logger().info('Server is busy, waiting...')

        self.command_number += 1

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Action status: {0}'.format(result.result))

        if self.command_number < len(self.commands):
            self.send_goal()
        elif self.command_number >= len(self.commands):
            sys.exit()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: odometry = {0}'.format(feedback.odom))



def main(args=None):
    rclpy.init(args=args)

    action_client = ActionTurtleClient(['forward', 'forward', 'turn_left', 'forward'])
    action_client.send_goal()

    rclpy.spin(action_client)

    rclpy.shutdown()



if __name__ == '__main__':
    main()
