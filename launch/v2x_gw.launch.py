import os
import urllib.parse
import socket

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python import get_package_share_directory
import launch.substitutions
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from launch_ros.actions import Node


from pathlib import Path

def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)



def generate_launch_description():

    #namespace = 'v2x'
    namespace = ''


    param_file = get_share_file(package_name='v2x_gw', file_name='config/v2x_gw.param.yaml')


    return launch.LaunchDescription([
        DeclareLaunchArgument('param', default_value=param_file, description='configuration file for v2x_gw.'),
        DeclareLaunchArgument('server.address_fusion', default_value='192.168.1.30', description='vehicleCAPTAIN routing core address.'),

        # V2X Gateway Node
        Node(
            package="v2x_gw",
            executable="v2x_gw",
            name="v2x_gw",
            parameters=[
                LaunchConfiguration('param'),
                {"server.address_fusion": LaunchConfiguration('server.address_fusion')}
                ],
            # prefix=['valgrind --leak-check=full'], #test for memory leaks
            namespace=namespace
        )

    ])
    

# code to resolve URI with python
def resolve_uri(uri: str) -> str:
    """
    Resolved the hostname of the given uri
    uri has to be in form scheme://hostname
    :raises ValueError if the uri is not valid, i.e. contains no hostname or IP address
    :param uri: the uri to resolve
    :return: resolved uri
    """
    uri_parts = urllib.parse.urlsplit(uri)
    if uri_parts.hostname is None:
        raise ValueError(f'Bad uri {uri}')
    ip_addr = socket.gethostbyname(uri_parts.hostname)
    uri_resolved = f'{uri_parts.scheme}://{ip_addr}:{uri_parts.port}'

    return uri_resolved

