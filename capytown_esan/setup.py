from setuptools import setup
import os
from glob import glob

package_name = 'capytown_esan'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='capytown',
    maintainer_email='codeplaigamessac@gmail.com',
    description='Scripts de movimiento para el robot Yahboom — ESAN',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'square = capytown_esan.square:main',
            'calibrate_beff = capytown_esan.calibrate_beff:main',
        ],
    },
)
