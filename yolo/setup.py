import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "params"), glob("params/*.yaml")),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.*')),
        (os.path.join('share', package_name, 'easy-yolov7'), glob('easy-yolov7/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ritwik',
    maintainer_email='ritwikrohan7@gmail.com',
    description='Nav2 stack and yolov7 for rosbot_xl',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_exec = yolo.webros:main',
            'detection = yolo.detection:main',
        ],
    },
)
