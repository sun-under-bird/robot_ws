from setuptools import setup
import os
from glob import glob

package_name = 'stereo_cam'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yahboom',
    maintainer_email='yahboom@todo.todo',
    description='拼接双目USB3.0相机节点',
    license='MIT',
    entry_points={
        'console_scripts': [
            'stereo_cam_node = stereo_cam.stereo_cam_node:main',
        ],
    },
)
