from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'camera_to_world_tf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robu',
    maintainer_email='Hauran21@htl-kaindorf.at',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_to_world=camera_to_world_tf.camera_to_world_node:main' 
        ],
    },
)

# ros2 pkg create --build-type ament_python camera_to_world_tf