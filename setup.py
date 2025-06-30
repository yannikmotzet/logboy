from setuptools import find_packages, setup
import glob

package_name = 'logboy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/assets', glob.glob('assets/*')),  # Use glob for assets
        ('share/' + package_name + '/config', glob.glob('config/*')),  # Use glob for config
        ('share/' + package_name + '/launch', glob.glob('launch/*')),  # Added launch folder
    ],
    install_requires=['setuptools', 'tkinter', 'pyyaml'],  # Added pyyaml
    zip_safe=True,
    maintainer='Yannik Motzet',
    maintainer_email='yannik.motzet@outlook.com',
    description='logboy: ROS2 .mcap recording tool',
    license='MIT',
    entry_points={
        'console_scripts': [
            'logboy_node = logboy.logboy_node:main',
            'logboy_gui_node = logboy.logboy_gui_node:main',
        ],
    },
)
