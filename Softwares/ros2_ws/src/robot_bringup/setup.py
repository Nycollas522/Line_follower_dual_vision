from glob import glob

from setuptools import setup

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name,
         ['package.xml']),
        ('share/' + package_name + '/launch',
         glob('launch/*.launch.py')),
        # O YAML precisa estar instalado para o launch achar via
        # get_package_share_directory. Faltava na versao anterior.
        ('share/' + package_name + '/config',
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='bolt',
    author_email='nycollasnascimento0peep@gmail.com',
    description='Seguidor de linha: percepcao, controle, servo e ponte ESP32',
    license='MIT',
    entry_points={
        'console_scripts': [
            'line_perception_node = '
            'robot_bringup.line_perception_node:main',
            'line_follower_node = robot_bringup.line_follower_node:main',
            'head_servo_node = robot_bringup.head_servo_node:main',
            'motor_serial_node = robot_bringup.motor_serial_node:main',
            'joy_teleop_node = robot_bringup.joy_teleop_node:main',
            'mode_manager_node = robot_bringup.mode_manager_node:main',
        ],
    },
)
