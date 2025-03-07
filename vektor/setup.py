from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vektor'

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
    maintainer='pi',
    maintainer_email='rammani@pi.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bot_direction = vektor.bot_direction:main',
            'r2_pressed_status = vektor.r2_pressed_status:main',
            'motor_rpm_target = vektor.motor_rpm_target:main',
            'uart_rpm_odom_publisher = vektor.uart_rpm_odom_publisher:main',
            'motor_control = vektor.motor_control:main',
        ],
    },
)
