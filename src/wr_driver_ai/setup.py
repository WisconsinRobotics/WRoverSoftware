from setuptools import find_packages, setup

package_name = 'wr_driver_ai'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'python-can'],
    zip_safe=True,
    maintainer='wiscrobo',
    maintainer_email='sungkar.bolat@gmail.com',
    description='Autonomously drives from one location to another',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "drive = wr_driver_ai.driver:main",
            "control_drive = wr_driver_ai.drive_control:main"
        ],
    },
)
