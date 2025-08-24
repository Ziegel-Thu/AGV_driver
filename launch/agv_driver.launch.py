from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取参数文件的路径
    config_file = os.path.join(
        get_package_share_directory('agv_driver'),
        'config',
        'agv_driver_params.yaml'
    )
    
    # 声明命令行参数，用于覆盖配置文件中的值
    control_port_arg = DeclareLaunchArgument(
        'control_port', 
        default_value='/dev/agv_ctrl',
        description='Control serial port device'
    )
    
    # 创建节点启动配置
    agv_driver_node = Node(
        package='agv_driver',
        executable='agv_driver_node',
        name='agv_driver',
        output='screen',
        parameters=[
            config_file,
            {
                'control_serial.port': LaunchConfiguration('control_port'),
            }
        ],
    )
    
    # 返回启动描述
    return LaunchDescription([
        control_port_arg,
        agv_driver_node
    ]) 