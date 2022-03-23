#!/usr/bin/env python

"""
 * Copyright (C) 2021 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
"""

import rospy
import guidance_plugin_validator

if __name__ == "__main__":
    """Main function to initialize the 'guidance_plugin_validator' node."""

    rospy.init_node("guidance_plugin_validator")

    enable_guidance_plugin_validator = rospy.get_param("~enable_guidance_plugin_validator", False)
    
    if enable_guidance_plugin_validator:
        try:       
            validator = guidance_plugin_validator.GuidancePluginValidator()
            validator.spin()
        except rospy.ROSInterruptException:
            pass