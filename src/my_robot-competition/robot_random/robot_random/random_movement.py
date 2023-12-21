import rclpy
from rclpy.node import Node
import random
from geometry_msgs.msg import Twist


class RandomPublisher(Node):

    def __init__(self):
        super().__init__('random_publisher')
        self.publisher_ = self.create_publisher(Twist, '/diff_drive/cmd_vel', 10)
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 1.0
        if random(-1.0,1.0) < 0:
            msg.angular.z = -1.0
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    random_publisher = RandomPublisher()

    rclpy.spin(random_publisher)

    random_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
