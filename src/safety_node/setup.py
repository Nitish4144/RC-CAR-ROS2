from setuptools import setup

package_name = 'manual_ctrl'

setup(
    name=package_name,
    packages=[package_name],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'joy_drive = manual_ctrl.joy_drive:main',
            'joy_raw = manual_ctrl.joy_raw:main',
            'motor_signals = manual_ctrl.motor_signals:main',
        ],
    },
)
