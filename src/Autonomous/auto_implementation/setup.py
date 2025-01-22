from setuptools import find_packages, setup

package_name = 'auto_implementation'

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
    maintainer='balabalu',
    maintainer_email='nicolasdittmarg1@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driving_logic = auto_implementation.driving_logic:main',
            'finding_aruco_tag = auto_implementation.finding_aruco_tag:main',
            'finding_object = auto_implementation.finding_object:main',
            'searching_drive = auto_implementation.searching_drive:main',
        ],
    },
)
