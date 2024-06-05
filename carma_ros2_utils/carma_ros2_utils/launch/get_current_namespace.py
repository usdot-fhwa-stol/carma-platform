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


from typing import Text

from launch.launch_context import LaunchContext
from launch.substitution import Substitution
from launch_ros.utilities import make_namespace_absolute
from rclpy.validate_namespace import validate_namespace
from launch.substitutions import SubstitutionFailure


class GetCurrentNamespace(Substitution):
    """
    Substitution that retrieves the current namespace 
    from the 'ros_namespace' launch configuration of the current context.
    This substitution will work correctly with the PushRosNamespace action if it is used.
    """

    def __init__(self) -> None:
        """
        Construct a get_current_namespace variable substitution.
        """
        super().__init__()

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return 'A get current namespace substitution.'

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by looking up the namespace in the launch context."""
        
        current_namespace = context.launch_configurations.get('ros_namespace', '/') # Get the current namespace from the launch context
        namespace = make_namespace_absolute(current_namespace) # Make the namespace absolute just to be safe

        try:
            validate_namespace(namespace) # Validate the namespace
        except Exception:
            raise SubstitutionFailure(
                'The current namespace could not be validated:. The computed value was {}'.format(namespace)
            )

        return namespace
