import rclpy
import numpy as np
from rclpy.node import Node


from geometry_msgs.msg import Twist
from std_msgs.msg import String


class Text_To_Cmd_Vel(Node):

    def __init__(self, turtle_speed, turtle_angular_speed):
        super().__init__('text_to_cmd_vel')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 1)
        self.subscriber_ = self.create_subscription(
            String,
            'cmd_text',
            self.listener_callback,
            1
        )
        self.subscriber_
        self.speed = turtle_speed
        self.angle = turtle_angular_speed

    def action(self, command):
        message = Twist()
        if command == "turn_left":
            message.angular.z = self.angle
        if command == "turn_right":
            message.angular.z = -self.angle
        if command == "move_forward":
            message.linear.x = self.speed
        if command == "move_backward":
            message.linear.x = -self.speed
        else:
            self.get_logger().info(f"Can't execute {command}.\nCommands list: turn_left, turn_right, move_forward, move_backward.")
        return message

    def execute_command(self, command):
        msg = self.action(command)
        self.publisher_.publish(msg)

    def listener_callback(self, msg):
	self.get_logger().info('I heard: "%s"' % msg.data)
        self.execute_command(msg.data)


def main(args=None):
    rclpy.init(args=args)

    text_to_cmd = Text_To_Cmd_Vel(1, 1.5)

    rclpy.spin(text_to_cmd)

    text_to_cmd.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
