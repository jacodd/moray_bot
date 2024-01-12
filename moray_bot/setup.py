from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'moray_bot'

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
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'models/turtlebot_waffle_gps'),
         glob('models/turtlebot_waffle_gps/*')),
         (os.path.join('share', package_name, 'rviz'), glob('rviz/*'))
    ],
    install_requires=['pip'],
    zip_safe=True,
    maintainer='ros_humble',
    maintainer_email='ros_humble@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follower = moray_bot.follower:main',
            
        ],
    },
)
