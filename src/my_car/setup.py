import glob
import os

from setuptools import find_packages, setup

package_name = 'my_car'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'sdf'), glob.glob('my_car/urdf/*.sdf')),
        (os.path.join('share', package_name, 'sdf'), glob.glob('my_car/urdf/*.xacro')),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         
            glob.glob('config/*.yaml') + glob.glob('config/*.rviz')),
        # nav2 的静态地图：.yaml 是参数、.pgm 是图本体，两个都得装
        (os.path.join('share', package_name, 'maps'),
            glob.glob('maps/*.yaml') + glob.glob('maps/*.pgm')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zrh',
    maintainer_email='zrh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'car_node = my_car.car_node:main',
            'teleop_node = my_car.teleop_node:main',
        ],
    },
)

