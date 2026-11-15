import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'target_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'LICENSE']),

        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tsikhan Nizavets',
    maintainer_email='tsikhannizavets@gmail.com',
    description='Detects targets in RGB images and publishes masked depth images',
    license='AGPL-3.0-only',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'target_detection = target_detection.target_detection:main'
        ],
    },
)
