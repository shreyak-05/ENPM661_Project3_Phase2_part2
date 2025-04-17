from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Setup Launch Parameters with realistic default values.
    start_position = LaunchConfiguration('start_position')
    end_position = LaunchConfiguration('end_position')
    robot_radius = LaunchConfiguration('robot_radius')
    clearance = LaunchConfiguration('clearance')
    delta_time = LaunchConfiguration('delta_time')
    goal_threshold = LaunchConfiguration('goal_threshold')
    wheel_radius = LaunchConfiguration('wheel_radius')
    wheel_distance = LaunchConfiguration('wheel_distance')
    rpms = LaunchConfiguration('rpms')

    start_position_launch_arg = DeclareLaunchArgument(
        'start_position',
        default_value='[17.0, 6.0, 0.0]',  # This is 150cm in x, 150cm in y - away from all walls
        description='Starting position of the robot [x, y, theta] in meters and degrees.'
    )
    end_position_launch_arg = DeclareLaunchArgument(
        'end_position',
        default_value='[21.0, 6.5, 0.0]',  # This is 350cm in x, 150cm in y - between walls at x=320cm and x=430cm
        description='Goal position of the robot [x, y, theta] in meters and degrees.'
    )
    robot_radius_launch_arg = DeclareLaunchArgument(
        'robot_radius',
        default_value='0.22',
        description='Robot radius in meters.'
    )
    clearance_launch_arg = DeclareLaunchArgument(
        'clearance',
        default_value='0.05',
        description='Required clearance around obstacles in meters.'
    )
    delta_time_launch_arg = DeclareLaunchArgument(
        'delta_time',
        default_value='1.0',
        description='Time step for simulation (seconds).'
    )
    goal_threshold_launch_arg = DeclareLaunchArgument(
        'goal_threshold',
        default_value='0.36',
        description='Threshold distance to consider the goal reached in meters.'
    )
    wheel_radius_launch_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.0335',
        description='Radius of the robot wheels in meters.'
    )
    wheel_distance_launch_arg = DeclareLaunchArgument(
        'wheel_distance',
        default_value='0.16',
        description='Distance between the wheels in meters.'
    )
    rpms_launch_arg = DeclareLaunchArgument(
        'rpms',
        default_value='[15.0, 30.0]',
        description='Wheel RPMs for the robot actions.'
    )
    # Optional static transform publisher to provide the "map" frame.
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'IMUSensor_BP_C_0'],
        output='screen'
    )

    #Create Process to Launch Falcon
    launch_falcon_sim = ExecuteProcess(
        cmd=[[
            ' ./Falcon.sh ',
            '-scenario=/home/shreya/duality/AStarPlanningProject/Scenarios/AMRPathPlanning/AMRPathPlanning.usda'
            ]],
        cwd="/home/shreya/duality/falconsim-v5.1.0216",
        shell=True
    )

    # Node to run the A* planner and control TurtleBot.
    astar_control_node = Node(
        package='astar_falcon_planner',
        executable='falcon_amr_controller',
        name='falcon_amr_controller',
        parameters=[{
            'start_position': start_position,
            'end_position': end_position,
            'robot_radius': robot_radius,
            'clearance': clearance,
            'delta_time': delta_time,
            'goal_threshold': goal_threshold,
            'wheel_radius': wheel_radius,
            'wheel_distance': wheel_distance,
            'rpms': rpms,
        }],
        output='screen'
    )

    return LaunchDescription([
        start_position_launch_arg,
        end_position_launch_arg,
        robot_radius_launch_arg,
        clearance_launch_arg,
        delta_time_launch_arg,
        goal_threshold_launch_arg,
        wheel_radius_launch_arg,
        wheel_distance_launch_arg,
        rpms_launch_arg,
        static_tf_node,      # Include this if the "map" frame is needed.
        astar_control_node,
        launch_falcon_sim,
    ])
