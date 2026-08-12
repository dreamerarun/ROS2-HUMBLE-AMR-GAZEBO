import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*.xacro'))),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*.rviz'))),
       (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
       (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
       (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml') + glob('maps/*.pgm')),  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arun',
    maintainer_email='arun@todo.todo',
    description='Custom robot URDF description package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
