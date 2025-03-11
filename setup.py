from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'thermal_image_colorizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dtc',
    maintainer_email='kabirkedia0111@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'colorizer = thermal_image_colorizer.colorizer:main',
            'flip_image = thermal_image_colorizer.flip_image:main'
        ],
    },
)
