from setuptools import find_packages, setup

# Set the package name to match the package.xml content
package_name = 'Manual_ctrl'

setup(
    name=package_name,
    version='0.0.0',
    # 1. Package Iterables: This iterable tells setuptools to find the
    # Python source files inside the inner 'rc_car_driver_pkg' directory.
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # 2. Data File Iterable: Simplified to match the requested style.
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    # Maintainer name format updated to match the example style
    maintainer='envisage',
    maintainer_email='envisage.smail.iitm.ac.in',
    description='ROS 2 package for the RC Car driver and teleoperation.',
    license='MIT',
    tests_require=['pytest'],
    # 3. Executable Script Iterables (console_scripts): These remain correct for your project.
    entry_points={
        'console_scripts': [
            # Node 1: The Joystick Controller
            'simple_car_controller = Manual_ctrl.simple_car_controller:main',
            # Node 2: The PWM Driver/Simulator
            'rc_car_pwm_simulator = Manual_ctrl.rc_car_pwm_driver:main',
        ],
    },
)
