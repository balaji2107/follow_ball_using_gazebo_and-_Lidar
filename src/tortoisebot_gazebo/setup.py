import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tortoisebot_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arz-1017',
    maintainer_email='arz-1017@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'laser_data = tortoisebot_gazebo.laserdata:main',
            'print_data = tortoisebot_gazebo.printdata:main',
            'filter_median = tortoisebot_gazebo.findmedian:main',
            'filter_moving_avg = tortoisebot_gazebo.movingaverage:main',
            'find_direction = tortoisebot_gazebo.finddirection:main'
        ],
    },
)
