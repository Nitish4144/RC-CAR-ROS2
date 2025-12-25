from setuptools import setup, find_packages

package_name = 'llm_ctrl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='driftpilot',
    maintainer_email='driftpilot@example.com',
    description='LLM based controller for RC car',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'llm_drive = llm_ctrl.llm_drive:main',
            'model  = llm_ctrl.controller:main',
        ],
    },
)

