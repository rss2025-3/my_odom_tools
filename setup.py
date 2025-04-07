from setuptools import setup, find_packages

package_name = 'my_odom_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(), # <- This must match the folder containing odom_republisher.py
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Odometry republisher',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_republisher = my_odom_tools.odom_republisher:main',
            'laser_scan_republisher = my_odom_tools.laser_scan_republisher:main',
        ],
    },
)
