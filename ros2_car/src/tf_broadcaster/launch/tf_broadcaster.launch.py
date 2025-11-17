import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # laser 坐标系到 base_link TF静态发布
    laser_tf_node = Node(
        package='tf2_ros',                                      # TF2 ROS2包
        executable='static_transform_publisher',                # 静态TF发布可执行程序 
        name='laser_broadcaster',                               # 节点名称
        arguments=[
            '0','0','0.05',                                     # x，y，z 偏移（单位：m）
            '0','0','0',                                        # roll，pitch，yaw 偏移 （单位 rad）
            'base_link','laser_link']                                # 父坐标系，子坐标系
    )


    # 获取启动Slam的launch文件路径
    cartographer_launch_file  = os.path.join(
        get_package_share_directory('tf_broadcaster'),
        'launch',
        'cartographer_2d.launch.py'
    )
    # 使用 IncludeLaunchDescription 包含另一个 launch 文件
    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cartographer_launch_file)
    )

     # 先启动静态 TF，再启动 Cartographer 节点
    return LaunchDescription([
        laser_tf_node,
        cartographer_launch
    ])