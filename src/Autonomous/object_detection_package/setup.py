from setuptools import find_packages, setup

package_name = 'object_detection_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['model.pt'])
    ],
    install_requires=[
      'setuptools',
      'rosidl_runtime_py',
      'ultralytics',
      'torch',
      'opencv-python'
    ],
    zip_safe=True,
    maintainer='David Wang',
    maintainer_email='davidw070129@gmail.com',
    description='A ROS2 package for YOLO-based object detection',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detection = object_detection_package.yolo_detection:main'
        ],
    },
)
