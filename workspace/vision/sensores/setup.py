from setuptools import setup

package_name = 'sensores'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ruy',
    maintainer_email='ruyrdz23@gmail.com',
    description='Paquete de detección con YOLO',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qr = sensores.qr:main',
            'hazmat = sensores.hazmat:main',
        ],
    },
)
