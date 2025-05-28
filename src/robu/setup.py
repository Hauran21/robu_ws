from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robu'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),#

        # Hier muss alles eingefügt werden was beim starten gestartet werden soll!!!!
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
            'remotectrl     = robu.ue06_remotectrl:main',
            'plf01_pub      = robu.plf01_ledstrip_pub:main',
            'plf01_sub      = robu.plf01_ledstrip_sub:main',
            'distance_sensor = robu.ue08_distance:main_distance_sensor',
            'obstacle_avoidance = robu.ue08_distance:main_obstacle_avoidance',
            'simple_kinematics = robu.ue09_turtlesim_simple_kinematics:main',
        ],
    },
)
