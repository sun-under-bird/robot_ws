from setuptools import find_packages, setup
import os
from glob  import glob
package_name = 'stereo_camera_pkg_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yahboom',
    maintainer_email='your.email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'v4l2 = stereo_camera_pkg_py.v4l2:main',
            'stereo_info = stereo_camera_pkg_py.stereo_info:main',
            'stereo_calib = stereo_camera_pkg_py.stereo_calib:main',
            'stereo_compressed = stereo_camera_pkg_py.stereo_compresseed:main',
        ],
    },
)
