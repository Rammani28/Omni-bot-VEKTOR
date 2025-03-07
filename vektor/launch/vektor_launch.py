import os
import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joy_node'
        ),
        launch_ros.actions.Node(
            package='vektor',
            executable='r2_pressed_status',
            name='r2_pressed_status_node'
        ),
        launch_ros.actions.Node(
            package='vektor',
            executable='bot_direction',
            name='bot_direction_node'
        ),
        launch_ros.actions.Node(
            package='vektor',
            executable='motor_rpm_target',
            name='motor_rpm_target_node'
        ),
        launch_ros.actions.Node(
            package='vektor',
            executable='motor_control',
            name='motor_control_node'
        ),
        launch_ros.actions.Node(
            package='vektor',
            executable='uart_rpm_odom_publisher',
            name='uart_rpm_odom_publisher_node'
        ),
    ])
