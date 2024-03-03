from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'opencv_tools'

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
    maintainer='jhon',
    maintainer_email='jhon_edw.gonzalez@uao.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'img_publisher = opencv_tools.image_publisher:main',
        'img_subscriber = opencv_tools.image_subscriber:main',
        'lowpass = opencv_tools.lowpass:main',
        'highpass = opencv_tools.highpass:main',
        'custom_filter = opencv_tools.custom_filter:main',
        'emotions = opencv_tools.emotions:main',
        'stats = opencv_tools.emotions_stats:main',
        
        ],
    },
)
