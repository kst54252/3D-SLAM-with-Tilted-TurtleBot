from setuptools import find_packages, setup

package_name = 'active_3d_core'

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
    maintainer='wanjunkim',
    maintainer_email='whanjunkim@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'keyboard_drive_node = active_3d_core.keyboard_drive_node:main',
            'cmd_vel_watch_node = active_3d_core.cmd_vel_watch_node:main',
            'odom_tf_node = active_3d_core.odom_tf_node:main',
            'spin_once_node = active_3d_core.spin_once_node:main',
            'tilted_scan_node = active_3d_core.tilted_scan_node:main',
        ],
    },
)
