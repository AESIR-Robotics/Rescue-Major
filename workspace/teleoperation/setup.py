from setuptools import find_packages, setup

package_name = 'Teleoperation'

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
    maintainer='martinorrego',
    maintainer_email='abraham.mata15012006@gmail.com"',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'reader_bag = GUI.reader_bag:main',
        'server= GUI.server:main'
        ],
    },
)
