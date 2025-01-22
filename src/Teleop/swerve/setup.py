from setuptools import find_packages, setup

package_name = 'swerve'

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
            'swerve_logic = swerve.swerve_logic:main',
            'rotate_motors = swerve.rotate_motors:main',
            'drive_motors = swerve.drive_motors:main',
        ],
    },
)
