from setuptools import find_packages, setup

package_name = 'parking_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sankalp',
    maintainer_email='sankalp@example.com',
    description='Parking controller for Ackermann vehicle',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'parking_controller_node = parking_controller.parking_controller_node:main',
        ],
    },
)
