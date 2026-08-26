# TODO: IMPLEMENT LAUNCH FILE

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    obstacles_arg = DeclareLaunchArgument('obstacles', default_value='')
    # bag_out_arg = DeclareLaunchArgument('bag_out', default_value='')

    # Get launch configurations
    obstacles = LaunchConfiguration('obstacles')
    # bag_out = LaunchConfiguration('bag_out')


    return LaunchDescription([
        obstacles_arg,
        # bag_out_arg,

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
        ),
        
        # ExecuteProcess(
        #     cmd=['ros2', 'bag', 'record', '-a', '-o', bag_out, '-s', 'sqlite3']
        # ),

        Node(
            package='circular',
            executable='draw_obst',
            name='draw_obstacles',
            parameters=[{
                'obstacles': obstacles
            }]
        ),

        Node(
            package='circular',
            executable='rrt_star',
            name='rrt_star',
            parameters=[{
                'obstacles': obstacles,
            }],
            # output='screen'
        ),
        
        Node(
            package='circular',
            executable='navigation',
            name='navigation',
            parameters=[{
                'obstacles': obstacles,
            }],
        ),

        Node(
            package="sim",
            executable="sim1",
            name="sim1"
        )
    ])