from launch import LaunchDescription
from launch_ros.actions import Node




def generate_launch_description():
   
    return LaunchDescription([
        #---------------------1---------------------------

        Node(
            package='motion_planning',
            executable='bfs',
            name='ss',
            output='screen',
            emulate_tty=True

        ),
        Node(
            package='motion_planning',
            executable='dfs_r',
            name='ss',
            output='screen',
            emulate_tty=True

        ),
        Node(
            package='motion_planning',
            executable='dijkstra',
            name='ss',
            output='screen',
            emulate_tty=True

        ),
        Node(
            package='motion_planning',
            executable='planner',
            name='ss',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'percent': 10}
            ]
        )
        
    ])