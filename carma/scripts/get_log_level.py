#!/usr/bin/env python3

# Copyright (C) 2021 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from typing import List
from typing import Text

import json

from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitution import Substitution


class GetLogLevel(Substitution):
    """
    Substitution that gets an checks for the log level of the provided package.

    The returned level will be debug, info, warning, error, or fatal.

    The input is a json string that contains a dictionary of package names and log levels.
    This can be provided by an environment variable or launch file argument.

    Example of intended usage:
    # A node launch action is provided with log level arguments and an EnvironmentVariable substitution provides the json string
    Node( ...,
        arguments=['--ros-args', '--log-level', GetLogLevel('node_package_name', EnvironmentVariable('CARMA_ROS_LOGGING_CONFIG'))]
    )
    # For composable nodes using the carma_ros2_utils component managers it looks like this
    ComposableNode(...,
        extra_arguments=[
            {'--log-level' : GetLogLevel('subsystem_controllers', EnvironmentVariable('CARMA_ROS_LOGGING_CONFIG')) }
        ],
    )

    """

    def __init__(
        self,
        package_name: SomeSubstitutionsType,
        json_dict: SomeSubstitutionsType
    ) -> None:
        """
        Construct a log level variable substitution.

        :param package_name: name of the package to check the log level for.
        :param json_dict: A json formatted dictionary that is a mapping of package names to log levels.
        """
        super().__init__()

        from launch.utilities import normalize_to_list_of_substitutions  # import here to avoid loop
        
        # Normalize the substitution inputs
        self.__package_name = normalize_to_list_of_substitutions(package_name)

        self.__json_dict = normalize_to_list_of_substitutions(json_dict)


    @property
    def package_name(self) -> List[Substitution]:
        """Getter for __package_name."""
        return self.__package_name

    @property
    def json_dict(self) -> List[Substitution]:
        """Getter for __json_dict."""
        return self.__json_dict


    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return 'A log level substitution'

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by looking up the package name in the json dictionary."""


        from launch.utilities import perform_substitutions  # import here to avoid loop

        json_dict = perform_substitutions(context, self.json_dict)
        package_name = perform_substitutions(context, self.package_name)
        
        return self.log_level_from_dict(package_name, json_dict)

    def log_level_from_dict(self, package, levels_json):
        """Helper method to get the log level from the json dictionary."""

        try:
            levels_json = levels_json.strip()
        except json.JSONDecodeError:
            print("The input to GetLogLevel was not a valid json string. Setting default log level WARN")
            return 'WARN'

        levels_dict = json.loads(levels_json)
        
        if (package in levels_dict.keys()):
            return levels_dict[package]

        elif ('default_level' in levels_dict):
            return levels_dict['default_level']

        else:
            print('The specified package was not found and not default_level set. Setting to WARN')
            return 'WARN'
