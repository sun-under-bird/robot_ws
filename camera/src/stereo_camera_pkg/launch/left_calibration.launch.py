from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 单目相机标定节点配置（适配ROS2 camera_calibration工具）
    mono_calibration_node = Node(
        package='camera_calibration',       # 标定工具功能包（需提前安装）
        executable='cameracalibrator',      # 标定工具可执行文件
        name='mono_calibrator',             # 节点名称（自定义）
        output='screen',                    # 日志输出到终端
        # 单目标定核心参数（根据你的标定板修改！）
        arguments=[
            '--size', '8x6',                # 标定板内角点数量（行×列，8×6方格填8x6）
            '--square', '0.02',             # 标定板方格尺寸（米，20mm=0.02m）
            '--no-service-check',           # 跳过service检查（避免找不到set_camera_info报错）
            # 单目标定仅需映射单个相机话题（关键区别：双目是left/right，单目是image）
            '--ros-args',
            '-r', '/image:=/left_camera/image_raw',  # 重映射：标定工具默认话题→你的相机图像话题
            # '-r', '/camera_info:=/left_camera/camera_info'  # 重映射相机内参话题（可选，增强兼容性）
        ]
    )

    return LaunchDescription([mono_calibration_node])