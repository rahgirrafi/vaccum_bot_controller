from setuptools import find_packages, setup

package_name = 'rpi_nodes'

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
    maintainer='vaccum-bot',
    maintainer_email='vaccum-bot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rpi_motor_dir = rpi_nodes.rpi_motor_dir:main',
            'keyboard_control = rpi_nodes.keyboard_control:main',
        ],
    },
)
