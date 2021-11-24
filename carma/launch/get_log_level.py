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

import json

def get_log_level(package, levels_json):

    levels_dict = json.loads(levels_json)
    if (package in levels_dict.keys()):
        return levels_dict[package]
    else:
        return levels_dict['default_level']


# Copyright 2018 Open Source Robotics Foundation, Inc.
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

"""Module for the EnvironmentVariable substitution."""

import os
from typing import Iterable
from typing import List
from typing import Optional
from typing import Text

import json

from launch.substitutions.substitution_failure import SubstitutionFailure
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitution import Substitution


class GetLogLevel(Substitution):
    """
    Substitution that gets an environment variable value as a string.

    If the environment variable is not found, it returns empty string.
    """

    def __init__(
        self,
        package_name: SomeSubstitutionsType,
        json_dict: SomeSubstitutionsType
    ) -> None:
        """
        Construct an enviroment variable substitution.

        :param name: name of the environment variable.
        :param default_value: used when the environment variable doesn't exist.
            If `None`, the substitution is not optional.
        :raise `SubstitutionFailure`:
            If the environment variable doesn't exist and `default_value` is `None`.
        """
        super().__init__()

        from launch.utilities import normalize_to_list_of_substitutions  # import here to avoid loop
        
        self.__package_name = normalize_to_list_of_substitutions(package_name)

        

        self.__json_dict = normalize_to_list_of_substitutions(json_dict)

        print('Length Json: ' + str(len(self.__json_dict)))

    @property
    def package_name(self) -> List[Substitution]:
        """Getter for name."""
        return self.__package_name

    @property
    def json_dict(self) -> List[Substitution]:
        """Getter for default_value."""
        return self.__json_dict


    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return 'I am a log level substitution' # TODO meaningful return

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by looking up the environment variable."""


        from launch.utilities import perform_substitutions  # import here to avoid loop

        json_dict = perform_substitutions(context, self.json_dict)
        package_name = perform_substitutions(context, self.package_name)
        print('Package name: ' + str(package_name))
        
        return self.log_level_from_dict(package_name, json_dict)

    def log_level_from_dict(self, package, levels_json):

        print('LogLevelsLow: ' + str(levels_json))
        levels_dict = json.loads(levels_json)
        if (package in levels_dict.keys()):
            return levels_dict[package]
        else:
            return levels_dict['default_level']
