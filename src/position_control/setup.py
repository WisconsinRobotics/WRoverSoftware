from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'position_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wiscrobo',
    maintainer_email='devansh.the.photofreak@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'send_position = position_control.send_position:main',
            'make_drawing = position_control.make_drawing:main',
            'run_arm = position_control.run_arm:main',
            'send_drawing = position_control.send_drawing:main',
        ],
    },
)
