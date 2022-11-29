# Copyright (C) 2022 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from carma_ros2_utils.launch.get_current_namespace import GetCurrentNamespace

import os
import subprocess



'''
This file is can be used to launch the CARMA carma_cloud_client_node.
  Though in carma-platform it may be launched directly from the base launch file.
'''

def generate_launch_description():


    REMOTE_USER="ubuntu"
    REMOTE_ADDR="www.carma-cloud.com"
    KEY_FILE="carma-cloud-test-1.pem"
    HOST_PORT="33333" # This port is forwarded to remote host (carma-cloud)
    REMOTE_PORT="10001" # This port is forwarded to local host 

    param_launch_path = os.path.join(
        get_package_share_directory('carma_cloud_client'), 'launch/scripts')
    
   
    cmd = param_launch_path + '/open_tunnels.sh'

    subprocess.check_call(['chmod','u+x', cmd])

    key_path = "/opt/carma/vehicle"
    key = key_path + '/' + KEY_FILE

    arg1 = '-u'
    arg2 = REMOTE_USER #'-a $REMOTE_ADDR'
    arg3 = '-a'
    arg4 = REMOTE_ADDR
    arg5 = '-k'
    arg6 = key
    arg7 = '-p'
    arg8 = REMOTE_PORT
    arg9 = '-r'
    arg10 = HOST_PORT

    subprocess.check_call(['sudo','chmod','400', key])
    subprocess.check_call(['sudo', cmd, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8,  arg9, arg10])

    # Declare the log_level launch argument
    log_level = LaunchConfiguration('log_level')
    declare_log_level_arg = DeclareLaunchArgument(
        name ='log_level', default_value='WARN')
    
    # Get parameter file path
    param_file_path = os.path.join(
        get_package_share_directory('carma_cloud_client'), 'config/parameters.yaml')

        
    # Launch node(s) in a carma container to allow logging to be configured
    container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='carma_cloud_client_container',
        namespace=GetCurrentNamespace(),
        executable='carma_component_container_mt',
        composable_node_descriptions=[
            
            # Launch the core node(s)
            ComposableNode(
                    package='carma_cloud_client',
                    plugin='carma_cloud_client::CarmaCloudClient',
                    name='carma_cloud_client',
                    extra_arguments=[
                        {'use_intra_process_comms': True},
                        {'--log-level' : log_level }
                    ],
                    parameters=[ param_file_path ]
            ),
        ]
    )

    return LaunchDescription([
        declare_log_level_arg,
        container
    ])
