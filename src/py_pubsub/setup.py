from setuptools import setup

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='havila',
    maintainer_email='dabburuhavila27@gmail.com',
    description='Simple ROS 2 pub-sub example',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'number_publisher = py_pubsub.number_publisher:main',
            'square_subscriber = py_pubsub.square_subscriber:main',
        ],
    },
)
