from setuptools import find_packages, setup

package_name = 'serial_motor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/serial_motor.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fastbot',
    maintainer_email='fastbot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	    'motor_driver = serial_motor.motor_driver:main',
        ],
    },
)
