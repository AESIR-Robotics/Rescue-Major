from setuptools import find_packages, setup

package_name = 'hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ruy',
    maintainer_email='ruy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "motors_node = hardware.motors_node:main",
            "motor_node = hardware.motor_node:main",
            "control_node = hardware.control_node:main",
            "odometry_node = hardware.odometry_node:main"
        ],
    },
)
