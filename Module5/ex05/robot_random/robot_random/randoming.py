import rclpy, sys
from rclpy.node import Node
import random
from geometry_msgs.msg import Twist


class RandomPublisher(Node):

    def __init__(self):
        super().__init__('publisher')
        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        timer_period = 0.5  
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ran = random.randrange(-10,10,1)
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 1.0
        if  ran < 0:
            msg.angular.z = -1.0
        self.publisher.publish(msg)

        
        
def main(args=None):
    rclpy.init(args=args)

    randoming = RandomPublisher()

    rclpy.spin(randoming)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
    