import os
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    rviz = os.path.join(get_package_share_directory('motion_planning'),'rviz','grid_search.rviz')
    
    


    return LaunchDescription([

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz],
            output={'both': 'log'}
        ),

        Node(
            package='motion_planning',
            executable='bfs',
            name='s1',
            output='screen',
            emulate_tty=True

        ),
        Node(
            package='motion_planning',
            executable='dfs',
            name='s2',
            output='screen',
            emulate_tty=True

        ),
        Node(
            package='motion_planning',
            executable='dijkstra',
            name='s3',
            output='screen',
            emulate_tty=True

        ),
        Node(
            package='motion_planning',
            executable='random',
            name='s4',
            output='screen',
            emulate_tty=True

        ),
        Node(
            package='motion_planning',
            executable='plot',
            name='s4',
            output='screen',
            emulate_tty=True

        ),
    ])

def main():

    generate_launch_description()

if __name__ == '__main__':
    main()
