from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim3',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            namespace='mimic',
            remappings=[
                ('/mimic/input/pose', '/turtlesim1/turtle1/pose'),
                ('/mimic/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel')]),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic2',
            namespace='mimic',
            remappings=[
                ('/mimic/input/pose', '/turtlesim2/turtle1/pose'),
                ('/mimic/output/cmd_vel', '/turtlesim3/turtle1/cmd_vel')])
    ])
