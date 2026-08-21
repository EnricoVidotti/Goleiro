# $ ros2 launch robot_bringup decision_bringup.launch.py


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import sys
sys.path.append("/home")
from robot_num import robot_number

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(name='log_level', default_value='info'))
    log_level = LaunchConfiguration('log_level')

    robot_config_file = 'robot_config'+str(robot_number)+'.yaml'
    control_config = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config',
        robot_config_file
    )

    # robot_number == 1 é o goleiro (mesmo critério de is_goalkeeper() em
    # decision_pkg_cpp/robot_behavior.cpp — hoje ROBOT_NUMBER é um #define
    # fixo em robot_behavior.hpp, não este parâmetro). A FSM do goleiro
    # (antes em goleiro_decision/goleiro_behavior_node) foi portada pra
    # dentro de decision_pkg_cpp/robot_behavior.cpp, então todo robô roda o
    # mesmo executável "robot_behavior"; só o goleiro precisa, além disso,
    # do goleiro_decision/localization_node (publica l_count/t_count/x_count
    # no tópico "robot_behavior" que a FSM do goleiro consome).
    #
    # goleiro_behavior_node NÃO é mais iniciado aqui: rodar os dois nós
    # (ele e robot_behavior) ao mesmo tempo faria duas decisões concorrentes
    # disputando o mesmo control_action.
    decision = Node(
        package="decision_pkg_cpp",
        executable="robot_behavior",
        output = 'screen',
        parameters = [control_config],
        arguments=['--ros-args', '--log-level', log_level,
                   '--log-level',  'rcl:=info',
                   '--log-level',  'rmw_fastrtps_cpp:=info'],
        emulate_tty=True
    )
    ld.add_action(decision)

    if robot_number == 1:
        goleiro_localization = Node(
            package="goleiro_decision",
            executable="localization_node",
            output = 'screen',
            parameters = [control_config],
            arguments=['--ros-args', '--log-level', log_level,
                       '--log-level',  'rcl:=info',
                       '--log-level',  'rmw_fastrtps_cpp:=info'],
            emulate_tty=True
        )
        ld.add_action(goleiro_localization)

    return ld