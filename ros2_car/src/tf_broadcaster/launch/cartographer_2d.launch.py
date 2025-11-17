from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Cartographer 2D 建图 launch 文件
    适用于 ROS 2 Foxy，包含：
    1. cartographer_node：SLAM 核心节点
    2. TF 静态发布器：laser -> base_link
    """

    # 获取 tf_broadcaster 包的共享目录
    pkg_share = get_package_share_directory('tf_broadcaster')

    # Lua 配置文件所在目录和文件名
    config_dir = os.path.join(pkg_share, 'config')
    config_basename = 'cartographer_2d.lua'

    # 文件存在检查
    lua_file = os.path.join(config_dir, config_basename)
    if not os.path.exists(lua_file):
        raise RuntimeError(f"Cartographer Lua 文件不存在: {lua_file}")

    
    cartographer_node = Node(package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],  
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', config_basename
        ],
        remappings=[
            ('/scan', '/diffbot/scan'),
            ('/odom', '/diffbot/odom')  
        ])

    # /map发布节点
    occupancy_grid_node = Node(package='cartographer_ros',
        executable='occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': True}])
    
    return LaunchDescription([
        cartographer_node,
        occupancy_grid_node
        # Cartographer 核心节点 - 修正版本
    ])