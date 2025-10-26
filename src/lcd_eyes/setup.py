from setuptools import setup

package_name = 'lcd_eyes'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'Pillow', 'adafruit-circuitpython-ssd1306'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='LCD eyes display node for RC car (ROS2)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'eye_display = lcd_eyes.eye_display:main',
        ],
    },
)

