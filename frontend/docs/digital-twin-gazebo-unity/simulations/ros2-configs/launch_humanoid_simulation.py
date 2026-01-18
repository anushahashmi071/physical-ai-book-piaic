from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Launch Gazebo with the humanoid world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('my_robot_description'),
                'worlds',
                'humanoid_world.world'
            ]),
            'verbose': 'false'
        }.items()
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'simple_humanoid',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0'
        ],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': """
                <?xml version="1.0"?>
                <robot name="simple_humanoid">
                  <link name="base_link">
                    <visual>
                      <geometry>
                        <box size="0.3 0.2 0.5"/>
                      </geometry>
                      <material name="gray">
                        <color rgba="0.8 0.8 0.8 1"/>
                      </material>
                    </visual>
                    <collision>
                      <geometry>
                        <box size="0.3 0.2 0.5"/>
                      </geometry>
                    </collision>
                    <inertial>
                      <mass value="10.0"/>
                      <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.6" iyz="0.0" izz="0.3"/>
                    </inertial>
                  </link>

                  <link name="torso">
                    <visual>
                      <geometry>
                        <box size="0.3 0.2 0.5"/>
                      </geometry>
                      <material name="light_gray">
                        <color rgba="0.7 0.7 0.7 1"/>
                      </material>
                    </visual>
                    <collision>
                      <geometry>
                        <box size="0.3 0.2 0.5"/>
                      </geometry>
                    </collision>
                    <inertial>
                      <mass value="5.0"/>
                      <inertia ixx="0.3" ixy="0.0" ixz="0.0" iyy="0.4" iyz="0.0" izz="0.2"/>
                    </inertial>
                  </link>

                  <joint name="base_to_torso" type="fixed">
                    <parent link="base_link"/>
                    <child link="torso"/>
                    <origin xyz="0 0 0.25"/>
                  </joint>

                  <link name="head">
                    <visual>
                      <geometry>
                        <sphere radius="0.1"/>
                      </geometry>
                      <material name="skin">
                        <color rgba="0.8 0.6 0.4 1"/>
                      </material>
                    </visual>
                    <collision>
                      <geometry>
                        <sphere radius="0.1"/>
                      </geometry>
                    </collision>
                    <inertial>
                      <mass value="2.0"/>
                      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
                    </inertial>
                  </link>

                  <joint name="neck_joint" type="revolute">
                    <parent link="torso"/>
                    <child link="head"/>
                    <origin xyz="0 0 0.3"/>
                    <axis xyz="0 0 1"/>
                    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
                  </joint>

                  <link name="left_thigh">
                    <visual>
                      <geometry>
                        <cylinder radius="0.05" length="0.4"/>
                      </geometry>
                      <material name="blue">
                        <color rgba="0.3 0.3 1.0 1"/>
                      </material>
                    </visual>
                    <collision>
                      <geometry>
                        <cylinder radius="0.05" length="0.4"/>
                      </geometry>
                    </collision>
                    <inertial>
                      <mass value="3.0"/>
                      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.01"/>
                    </inertial>
                  </link>

                  <joint name="left_hip_joint" type="revolute">
                    <parent link="torso"/>
                    <child link="left_thigh"/>
                    <origin xyz="-0.1 0 0.2"/>
                    <axis xyz="0 1 0"/>
                    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
                  </joint>

                  <link name="right_thigh">
                    <visual>
                      <geometry>
                        <cylinder radius="0.05" length="0.4"/>
                      </geometry>
                      <material name="blue">
                        <color rgba="0.3 0.3 1.0 1"/>
                      </material>
                    </visual>
                    <collision>
                      <geometry>
                        <cylinder radius="0.05" length="0.4"/>
                      </geometry>
                    </collision>
                    <inertial>
                      <mass value="3.0"/>
                      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.01"/>
                    </inertial>
                  </link>

                  <joint name="right_hip_joint" type="revolute">
                    <parent link="torso"/>
                    <child link="right_thigh"/>
                    <origin xyz="0.1 0 0.2"/>
                    <axis xyz="0 1 0"/>
                    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
                  </joint>
                </robot>
            """
        }]
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('/joint_states', 'joint_states')
        ]
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add the actions
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)

    return ld