import os
from glob import glob
from setuptools import setup

package_name = 'autopatrol_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+"/config", ['config/patrol_config.yaml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fishros',
    maintainer_email='fish@fishros.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'patrol_node=autopatrol_robot.patrol_node:main',
            'speaker=autopatrol_robot.speaker:main',
            'param_test=autopatrol_robot.param_test:main',
        ],
    },
)
