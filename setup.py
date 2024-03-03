from setuptools import setup
import os
from glob import glob

package_name = 'pointcloud_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hiroselab',
    maintainer_email='hiroselab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'centerpoints = pointcloud_package.centerpoints:main',
            'clusteringpoints = pointcloud_package.clusteringpoints:main',
            'trackingpoints = pointcloud_package.trackingpoints:main',
            'matchingpoints = pointcloud_package.matchingpoints:main',
        ],
    },
)
