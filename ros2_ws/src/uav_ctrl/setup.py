from setuptools import setup
import os
from glob import glob

package_name = 'uav_ctrl'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.yaml')),
    ],
    install_requires=[],
    zip_safe=True,
    maintainer='Patrick',
    maintainer_email='you@example.com',
    description='Mod drone package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lala_test = uav_ctrl.lala_test:main',
            'offboard = uav_ctrl.offboard:main',
        ],
    },
)