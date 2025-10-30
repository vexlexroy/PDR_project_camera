from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'camera_test_gt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leonardo',
    maintainer_email='leonardo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_pub_node = camera_test_gt.camera_pub_node:main',
            'gt_test_detector_node = camera_test_gt.gt_test_detector_node:main',
        ],
    },
)
