from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="uav_camera_receiver",
                executable="image_decompressor_node",
                name="color_image_decompressor",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/uav/camera/color/image_raw/compressed",
                        "output_topic": "/main/uav/camera/color/image_raw",
                        "encoding": "bgr8",
                    }
                ],
            ),
            Node(
                package="uav_camera_receiver",
                executable="image_decompressor_node",
                name="depth_image_decompressor",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/uav/camera/depth/image_rect_raw/compressed",
                        "output_topic": "/main/uav/camera/depth/image_rect_raw",
                        "encoding": "16UC1",
                    }
                ],
            ),
        ]
    )
