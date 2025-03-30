from setuptools import find_packages, setup

package_name = 'position_control'

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
    maintainer='wiscrobo',
    maintainer_email='devansh.the.photofreak@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'send_position = position_control.send_position:main',
            'make_drawing = position_control.make_drawing:main',
        ],
    },
)
