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

'''
Generates a json dictionary of package names with log levels.
The default level will be at key default_level.
Possible levels [ debug, info, warn, error, fatal ]
param: config_file_path The path to a log4j format .conf file.
       relevent log level lines should have the form log4j.logger.ros.<package_name>=<log_level>
'''
def generate_log_levels_impl(config_file_path):

    levels = { 'default_level' : 'WARN' } # Default log level will be WARN

    # Open the config file and parse its contents
    with open(config_file_path, 'r') as config_file:

        for line in config_file:
            
            no_ws_line = "".join(line.split()) # Remove white space

            if not no_ws_line.startswith('log4j'): # If this line is not a log configuration line then continue
                continue

            parts = no_ws_line.split('=') # Separate the logger name from the log level
            
            if len(parts) != 2:
                print("Failed to process line: " + str(no_ws_line))
                continue

            full_logger_package = parts[0]
            log_level = parts[1]

            package_parts = full_logger_package.split('.')

            if (len(package_parts) < 3): # We are expecting a format of log4j.logger.ros.<package_name>
                print("Failed to process line: " + str(no_ws_line))

            # If this line is the top ros level log then update the default value based on the provided log level
            if (len(package_parts) == 3 and package_parts[2] == 'ros'):
                levels['default_level'] = log_level
                continue

            elif(len(package_parts) >= 4): # This is a package log level descripter so get the package name
                package = ''
                for part in package_parts[3:]:
                    package += part
                    package += '.'
                package = package[:-1]
                levels[ package ] = log_level

            else:
                print("Failed to process line: " + str(no_ws_line))
                continue

    return levels

def generate_log_levels(config_file_path):
    return str(generate_log_levels_impl(config_file_path)).replace("'", '"') # Convert dictionary to string and use double quotes instead of single for valid json format
