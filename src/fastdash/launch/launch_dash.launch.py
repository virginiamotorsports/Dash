from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fastdash',
            executable='fastdash',
            namespace="",
            name='datalogger',
            # Launch the node with root access (GPIO) in a shell
            # prefix=["sudo -E env \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\"  bash -c "],
            # shell=True,
        ),
        Node(
            package='pydash',
            executable='gui',
            namespace="",
            name='gui',
            # Launch the node with root access (GPIO) in a shell
            additional_env={'DISPLAY': ':0'},
            # prefix=["\"DISPLAY=:0\" \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\"  bash -c "],
            shell=True,
        ),
    ])