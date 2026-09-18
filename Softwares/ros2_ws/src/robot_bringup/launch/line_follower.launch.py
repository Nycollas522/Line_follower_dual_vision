#!/usr/bin/env python3
"""Bringup do seguidor de linha.

Argumentos uteis:
  cameras:=false     nao sobe as cameras (util com rosbag/replay)
  serial:=false      nao sobe a ponte serial (validacao de visao sem robo)
  preview:=false     nao sobe a camera superior nem o servo
  port:=/dev/ttyUSB0 porta do ESP32

Todos os nos recebem o MESMO arquivo de parametros. Os nomes dos nos no
launch batem com as chaves do YAML -- e por isso que os dois nos de
percepcao, que rodam o mesmo executavel, precisam de nomes distintos
(line_perception_bottom / line_perception_front).
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Monta o conjunto de nos do seguidor de linha."""
    package_share = get_package_share_directory('robot_bringup')
    default_params = os.path.join(package_share, 'config', 'line_follower.yaml')

    params_file = LaunchConfiguration('params_file')
    use_cameras = LaunchConfiguration('cameras')
    use_serial = LaunchConfiguration('serial')
    use_preview = LaunchConfiguration('preview')
    port = LaunchConfiguration('port')

    arguments = [
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='YAML com os parametros de todos os nos',
        ),
        DeclareLaunchArgument(
            'cameras', default_value='true',
            description='Sobe os nos de camera (camera_ros)',
        ),
        DeclareLaunchArgument(
            'serial', default_value='true',
            description='Sobe a ponte serial com o ESP32',
        ),
        DeclareLaunchArgument(
            'preview', default_value='true',
            description='Sobe a camera superior e o no do servo',
        ),
        DeclareLaunchArgument(
            'port', default_value='/dev/ttyACM0',
            description='Porta serial do ESP32',
        ),
        DeclareLaunchArgument(
            'cam_width', default_value='640',
            description='Largura publicada pelas cameras',
        ),
        DeclareLaunchArgument(
            'cam_height', default_value='480',
            description='Altura publicada pelas cameras',
        ),
        DeclareLaunchArgument(
            'joy', default_value='true',
            description='Sobe o leitor de joystick e o teleop de RC',
        ),
        DeclareLaunchArgument(
            'joy_dev', default_value='/dev/input/js0',
            description='Dispositivo do joystick',
        ),
    ]

    # RESOLUCAO DAS CAMERAS.
    #
    # O detector reduz para work_width (160 px) na PRIMEIRA operacao do
    # processamento -- 640x480 sao 307200 px dos quais ele usa ~19200.
    # Os 16x restantes sao serializados, transportados e convertidos
    # para serem jogados fora.
    #
    # ATENCAO: mudar a resolucao so e seguro se o campo de visao for
    # PRESERVADO. Se o libcamera escolher um modo de sensor que CORTA em
    # vez de escalar, toda a calibracao geometrica (depth_*, width_*)
    # passa a mentir. O teste e a largura reportada da fita: ela tem
    # 20 mm de verdade, entao line_width tem de continuar em ~20.
    cam_w = LaunchConfiguration('cam_width')
    cam_h = LaunchConfiguration('cam_height')

    cameras = GroupAction(
        condition=IfCondition(use_cameras),
        actions=[
            # Camera inferior: percepcao primaria.
            Node(
                package='camera_ros',
                executable='camera_node',
                name='camera_bottom',
                parameters=[{'camera': 1, 'width': cam_w, 'height': cam_h}],
                remappings=[('~/image_raw', '/cam_bottom/image_raw'),
                            ('~/camera_info', '/cam_bottom/camera_info')],
                output='screen',
            ),
            # Camera superior (no servo): preview.
            Node(
                package='camera_ros',
                executable='camera_node',
                name='camera_front',
                condition=IfCondition(use_preview),
                parameters=[{'camera': 0, 'width': cam_w, 'height': cam_h}],
                remappings=[('~/image_raw', '/cam_front/image_raw'),
                            ('~/camera_info', '/cam_front/camera_info')],
                output='screen',
            ),
        ],
    )

    perception_bottom = Node(
        package='robot_bringup',
        executable='line_perception_node',
        name='line_perception_bottom',
        parameters=[params_file],
        output='screen',
    )

    perception_front = Node(
        package='robot_bringup',
        executable='line_perception_node',
        name='line_perception_front',
        condition=IfCondition(use_preview),
        parameters=[params_file],
        output='screen',
    )

    follower = Node(
        package='robot_bringup',
        executable='line_follower_node',
        name='line_follower_node',
        parameters=[params_file],
        output='screen',
    )

    head = Node(
        package='robot_bringup',
        executable='head_servo_node',
        name='head_servo_node',
        condition=IfCondition(use_preview),
        parameters=[params_file],
        output='screen',
    )

    use_joy = LaunchConfiguration('joy')
    joy_dev = LaunchConfiguration('joy_dev')

    serial_bridge = Node(
        package='robot_bringup',
        executable='motor_serial_node',
        name='motor_serial_node',
        condition=IfCondition(use_serial),
        parameters=[params_file, {'port': port}],
        output='screen',
    )

    # GERENTE DE MODO. Traduz o que foi escolhido no menu do ESP32 em
    # parametros nos demais nos. Sempre no ar: e ele que permite trocar
    # entre seguidor e RC sem derrubar processo nenhum -- e derrubar
    # processo derrubaria as cameras junto, que o operador quer ver por
    # rqt nos DOIS modos.
    mode_manager = Node(
        package='robot_bringup',
        executable='mode_manager_node',
        name='mode_manager_node',
        parameters=[params_file],
        output='screen',
    )

    # Leitor do joystick. Fica no ar mesmo sem controle pareado: sem
    # /dev/input/js0 ele apenas nao publica, e assim o modo RC funciona
    # assim que o controle conecta, sem precisar relancar nada.
    joy_reader = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        condition=IfCondition(use_joy),
        parameters=[{'dev': joy_dev, 'autorepeat_rate': 20.0,
                     'deadzone': 0.0}],
        output='screen',
    )

    # Tradutor joystick -> /cmd_vel. So age no modo RC (ver
    # exigir_modo_rc), entao conviver com o seguidor e seguro.
    joy_teleop = Node(
        package='robot_bringup',
        executable='joy_teleop_node',
        name='joy_teleop_node',
        condition=IfCondition(use_joy),
        parameters=[params_file],
        output='screen',
    )

    return LaunchDescription(
        arguments
        + [cameras, perception_bottom, perception_front, follower, head,
           serial_bridge, mode_manager, joy_reader, joy_teleop]
    )
