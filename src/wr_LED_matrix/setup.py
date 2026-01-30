from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wr_LED_matrix'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wiscrobo',
    maintainer_email='devansh.the.photofreak@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'led_service = wr_LED_matrix.led_service:main',
            'led_party_client = wr_LED_matrix.led_party_client:main',
        ],
    },
)
