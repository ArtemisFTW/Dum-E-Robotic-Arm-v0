from glob import glob
from setuptools import find_packages, setup

setup(
    name='dum_e_control', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/dum_e_control']),
        ('share/dum_e_control', ['package.xml']),
        ('share/dum_e_control/launch', glob('launch/*.launch.py')),
        ('share/dum_e_control/config', glob('config/*')),
    ],
    package_data={'dum_e_control.core': ['robot_config.json']},
    install_requires=['setuptools', 'numpy', 'pyserial', 'PyYAML'],
    tests_require=['pytest'],
    zip_safe=False,
    maintainer='Dum-E maintainers', maintainer_email='maintainer@example.com',
    description='ROS 2 joint calculations and serial control for Dum-E',
    license='Unspecified',
    entry_points={'console_scripts': [
        'kinematics_node = dum_e_control.kinematics_node:main',
        'serial_bridge = dum_e_control.serial_bridge:main',
        'arm = dum_e_control.command:main',
        'gui = dum_e_control.gui:main',
    ]},
)
