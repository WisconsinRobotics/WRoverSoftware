from setuptools import find_packages, setup

package_name = 'arm_test_python'

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
    maintainer='test',
    maintainer_email='nicolasdittmarg1@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_test_logic = arm_test_python.arm_test_logic:main',
            'arm_test_neo = arm_test_python.arm_test_neo:main',
            'ik_subscriber = arm_test_python.ik_subscriber:main',
            'rail_subscriber = arm_test_python.rail_subscriber:main',
            'science_logic = arm_test_python.science_logic:main',
            'science_logic = arm_test_python.science_neo:main',
        ],
    },
)
