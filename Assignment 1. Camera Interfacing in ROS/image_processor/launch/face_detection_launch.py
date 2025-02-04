from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the USB camera node and remap `/image_raw` to `/camera1/image_raw`
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            remappings=[
                ('/image_raw', '/camera1/image_raw')  # ✅ Remap the topic
            ]
        ),

        # Start the face detection node (now correctly listening to `/camera1/image_raw`)
        Node(
            package='image_processor',
            executable='face_detection',
            name='face_detection',
            output='screen'
        ),

        # Start rqt_image_view for visualization
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='image_view',
            output='screen'
        ),
    ])


