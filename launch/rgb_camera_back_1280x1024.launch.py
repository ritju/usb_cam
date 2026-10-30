import os
import json
from pathlib import Path
import sys

# Hack to get relative import of .camera_config file working
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)

from camera_config import CameraConfig, USB_CAM_DIR

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import SetEnvironmentVariable


def parse_cameras():
    # 从环境变量获取 CAMERAS 值
    # cameras_str = os.getenv('CAMERAS')
    
    cameras_str = '[{"name":"rgb_camera_back", "config_file":"back_1280x1024.yaml"}]'

    if not cameras_str:
        print("错误：CAMERAS环境变量未设置")
        return None

    # 去除可能的单引号包裹
    if cameras_str.startswith("'") and cameras_str.endswith("'"):
        cameras_str = cameras_str[1:-1]

    try:
        cameras_data = json.loads(cameras_str)

        if not isinstance(cameras_data, list):
            print("错误：CAMERAS不是一个列表")
            return None

        for camera in cameras_data:
            if not all(key in camera for key in ['name', 'config_file']):
                print(f"错误：相机配置缺少必要字段 - {camera}")
                return None

        return cameras_data

    except json.JSONDecodeError as e:
        print(f"JSON解析错误: {e}")
        return None


CAMERAS = []

camera_infos = parse_cameras()

if camera_infos is not None:
    for camera_info in camera_infos:
        print(f'name: {camera_info["name"]}')
        print(f'config_file: {camera_info["config_file"]}')

        CAMERAS.append(
            CameraConfig(
                name=camera_info["name"],
                param_path=Path(
                    '/map/config/usb_cam',
                    camera_info["config_file"]
                )
            )
        )


def generate_launch_description():
    ld = LaunchDescription()

    if os.getenv('USB_CAM_UNBUFFERED_LOG', '0').lower() in ('1', 'true', 'yes'):
        ld.add_action(
            SetEnvironmentVariable(
                'RCUTILS_LOGGING_BUFFERED_STREAM', '0'
            )
        )

    camera_nodes = [
        ComposableNode(
            package='usb_cam',
            plugin='usb_cam::UsbCamNode',
            name=camera.name,
            namespace=camera.namespace,
            parameters=[str(camera.param_path)],
            remappings=camera.remappings,
        )
        for camera in CAMERAS
    ]
    
    # ImageStreamNode 参数
    camera_back = os.environ.get(
        "TOPIC_CAMERA_BACK", "rgb_camera_back/image_raw"
    )

    video_width_back = int(os.environ.get("VIDEO_WIDTH_BACK", "640"))
    video_height_back = int(os.environ.get("VIDEO_HEIGHT_BACK", "480"))

    camera_back_is_compressed = (
        os.environ.get("TOPIC_CAMERA_BACK_IS_COMPRESSED", "False") == "True"
    )

    disk_usage_threshold = float(
        os.environ.get("DISK_USAGE_THRESHOLD", "99.5")
    )
    
    # 新增 ImageStreamNode 组件
    camera_nodes.append(
        ComposableNode(
            package='cpp_video_stream',
            plugin='ImageStreamNode',
            name='cpp_image_stream_node_back',
            parameters=[{
                'stream_topic': camera_back,
                'is_compressed_image': camera_back_is_compressed,
                'stream_enable_topic': 'camera3/stream_enable',
                'stream_name': 'camera3',
                'video_dir': '/video_data/camera3/videos_cpp',
                'video_width': video_width_back,
                'video_height': video_height_back,
                'disk_usage_threshold': disk_usage_threshold,
            }]
        )
    )

    camera_container = ComposableNodeContainer(
        name='usb_cam_with_image_stream_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=camera_nodes,
        output='screen',
        respawn=True,
        respawn_delay=10,
    )
    ld.add_action(camera_container)

    return ld