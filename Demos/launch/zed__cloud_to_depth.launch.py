import os
import yaml
import pathlib
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch import LaunchContext

'''
Used to load parameters for composable nodes
'''
def dump_params(path, name):
    import yaml
    with open(path, 'r') as file:
        y = yaml.safe_load(file)[name]['ros__parameters']
        return y

param_path = os.path.join(
    get_package_share_directory("native_adapters_demos"), 
    'params', 
    'zed_demo.yaml')

def generate_launch_description():


    configured_params = RewrittenYaml(
        source_file=param_path,
        root_key='',
        param_rewrites={},
        convert_types=True)

    lc = LaunchContext()
    conf_path = configured_params.perform(lc)

    return LaunchDescription([
        Node(
            name='base2cam_tf',
            package = "tf2_ros", 
            executable = "static_transform_publisher",
            arguments = ["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "1.0", "world", "camera"],
        ),
        ComposableNodeContainer(
            name='demo_container',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='hal_stereolabs_zed2',
                    plugin='hal::StereolabsZed2',
                    name='hal_stereolabs_zed2',
                    parameters=[dump_params(conf_path, "hal_stereolabs_zed2")],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                ComposableNode(
                    package='cloud_as_depth',
                    plugin='perception::CloudToDepth',
                    name='cloud_to_depth',
                    parameters=[dump_params(conf_path, "cloud_to_depth")],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
            ],
            output={
                            "stdout": "screen",
                            "stderr": "screen",
            },
    )])