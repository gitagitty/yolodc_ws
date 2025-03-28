from setuptools import setup

package_name = 'yolo_realsense_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='evan',
    maintainer_email='chenyu1056@gmail.com',
    description='YOLO with RealSense D415i - Python Publisher',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_realsense_node = yolo_realsense_py.yolo_realsense_node:main',
        ],
    },
)
