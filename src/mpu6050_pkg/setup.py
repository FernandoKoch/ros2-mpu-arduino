from setuptools import find_packages, setup

package_name = 'mpu6050_pkg'

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
    maintainer='fernandocarlos',
    maintainer_email='fernandocarlos@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "mpu6050_node = mpu6050_pkg.mpu6050_node:main",
            "mpu6050_filter_node = mpu6050_pkg.mpu6050_filter_node:main",
            "mpu6050_serial_node = mpu6050_pkg.mpu6050_serial_node:main",
            "imu_subscriber = mpu6050_pkg.imu_subscriber:main"
        ],
    },
)
