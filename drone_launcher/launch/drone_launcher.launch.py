from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Inicia MAVProxy
        ExecuteProcess(
            cmd=[
                'mavproxy.py',
                '--master=/dev/ttyAMA0',
                '--baudrate=921600'
            ],
            output='screen',
        ),

        # Espera 5 segundos antes de iniciar MAVROS (puedes ajustar el tiempo)
        TimerAction(
            period=5.0,  # Tiempo de espera en segundos
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2',
                        'launch',
                        'mavros',
                        'px4.launch',
                        'fcu_url:=serial:///dev/ttyAMA0:921600'
                    ],
                    output='screen',
                )
            ],
        ),
    ])
