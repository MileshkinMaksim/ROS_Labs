import rclpy
import sys
import numpy as np
import time

from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class MoveToGoal(Node):
    def __init__(self, x, y, theta, turtle_number):
        super().__init__('move_to_goal')
        self.dest_x = x
        self.dest_y = y
        self.dest_theta = theta

        if theta < 0:
            self.dest_theta = 2 * np.pi - theta

        self.not_moved = True

        self.publisher_ = self.create_publisher(
            Twist,
            f'/turtle{turtle_number}/cmd_vel',
            10
        )

        self.subscriber_ = self.create_subscription(
            Pose,
            f'/turtle{turtle_number}/pose',
            self.turtle_pose_callback,
            1
        )

    def rotation_angle(self, dest_vector):
        rotation_angle = self.turtle_pose.theta  # current angle

        basis = np.array([1, 0])
        dest_vector_angle = 2. * np.pi - np.arccos(np.dot(dest_vector, basis) / (np.linalg.norm(dest_vector) * np.linalg.norm(basis)))

        if dest_vector[1] >= 0:
            dest_vector_angle = np.arccos(np.dot(dest_vector, basis) / (np.linalg.norm(dest_vector) * np.linalg.norm(basis)))

        turtle_angle = dest_vector_angle
        # сделано так, что сначала якобы вектор basis лежит под углом theta_turtle к оси Ox,
        # затем мы поворачиваем его так, чтобы он лежал на положительной полуоси Ox,
        # и находим угол между ним и вектором направления черепахи
        dest_vector_angle -= rotation_angle
        if dest_vector_angle >= 2 * np.pi:
            dest_vector_angle -= 2 * np.pi
        elif dest_vector_angle < 0:
            dest_vector_angle += 2 * np.pi

        rotation_angle = dest_vector_angle
        if rotation_angle > np.pi:
            rotation_angle = -(2 * np.pi - dest_vector_angle)

        return rotation_angle, turtle_angle

    def move_turtle(self):
        # calculate new theta
        x_turtle, y_turtle = self.turtle_pose.x, self.turtle_pose.y

        dest_vector = np.array([self.dest_x - x_turtle, self.dest_y - y_turtle])
        speed_value = np.linalg.norm(dest_vector)
        rotation_angle, turtle_angle = self.rotation_angle(dest_vector)

        # rotate
        msg1 = Twist()
        msg1.angular.z = rotation_angle
        self.publisher_.publish(msg1)
        time.sleep(2)

        # move to (x, y) coordinate
        msg2 = Twist()
        msg2.linear.x = speed_value
        self.publisher_.publish(msg2)
        time.sleep(2)

        # rotate to angle theta
        rotate_to_theta_angle = self.dest_theta - turtle_angle
        if rotate_to_theta_angle > np.pi:
            rotate_to_theta_angle = -(2 * np.pi - rotate_to_theta_angle)
        msg3 = Twist()
        msg3.angular.z = rotate_to_theta_angle
        self.publisher_.publish(msg3)
        self.not_moved = False

    def turtle_pose_callback(self, msg):

        self.turtle_pose = msg

        if self.not_moved:
            self.get_logger().info(f'Position received: {msg.x}, {msg.y}, {msg.theta}')
            self.move_turtle()
            self.destroy_node()
            sys.exit()


def main(args=None):
    rclpy.init(args=args)

    # аргументы командной строки: x, y - финальные координаты черпахи, theta - угол поворота черепахи в финальной точке
    x, y, theta = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])

    process = MoveToGoal(x, y, theta, 1) # 4й  аргумент - номер черепахи

    rclpy.spin(process)

    process.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

