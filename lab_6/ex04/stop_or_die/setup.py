from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'stop_or_die'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ruby',
    maintainer_email='dimaskrut71@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "sod_l = stop_or_die.stop_sub_lidar:main",
            "sod_d = stop_or_die.stop_sub_depth:main",
        ],
    },
)
