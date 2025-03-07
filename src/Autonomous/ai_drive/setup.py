from setuptools import find_packages, setup

package_name = 'ai_drive'

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
    maintainer='pranitvaikuntam',
    maintainer_email='pranitvaikuntam@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "basic_movement = ai_drive.basic_movement:main", 
            "sensor = ai_drive.sensor:main"
        ],
    },
)
