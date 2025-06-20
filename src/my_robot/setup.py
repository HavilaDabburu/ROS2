from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'my_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[

        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),


        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),

        (os.path.join('share', package_name, 'urdf'),
         glob('urdf/*.xacro')),
	(os.path.join('share', package_name, 'world'), glob('world/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A 4-wheeled skid-steer robot with sensors, URDF/Xacro, Gazebo, and RViz support',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stopper_node = my_robot.stopper_node:main',
            'starter_node = my_robot.starter_node:main',
],
    },
)
