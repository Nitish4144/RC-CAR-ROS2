from setuptools import setup
import os
from glob import glob

package_name = 'oled_eyes'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='manoj',
    maintainer_email='your@email.com',
    description='OLED eyes that follow Ackermann steering and throttle.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'oled_eyes_node = oled_eyes.oled_eyes_node:main'
        ],
    },
)
