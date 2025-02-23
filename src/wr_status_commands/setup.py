from setuptools import find_packages, setup

package_name = 'wr_status_commands'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Waits for the can0 canbus to publish a bitarray of data, then graps the associated status_command to publish the correct data. ',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'status_commands = wr_status_commands.status_commands:main'
        ],
    },
)
