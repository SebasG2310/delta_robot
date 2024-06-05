from setuptools import find_packages, setup

package_name = 'delta_robot'

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
    maintainer='sebas2310',
    maintainer_email='gomez.sebastian2310@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ik_node = delta_robot.ik_node:main',
            'inverse_kinematics = delta_robot.inverse_kinematics:main',
            'motor_node = delta_robot.motor_node:main',
            'gui = delta_robot.gui:main',
            'delta_GUI = delta_robot.delta_GUI:main',
        ],
    },
)
