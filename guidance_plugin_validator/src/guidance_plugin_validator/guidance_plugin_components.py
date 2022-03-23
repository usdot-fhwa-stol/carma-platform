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
from cav_msgs.msg import Plugin 

class StrategicPluginResults:
    """
    Convenience class used to store the validation results (both required and optional) 
    for a Guidance Strategic Plugin. 
    """

    def __init__(self, plugin_name):
        """Constructor for StrategicPluginResults"""

        # Set the plugin's name, as used by the plugin_discovery topics and the plugin's advertised service(s)
        self.plugin_name = plugin_name
        
        # Set the plugin's node name
        if plugin_name == "RouteFollowing":
            self.node_name = "/guidance/route_following_plugin"
        else:
            rospy.logerr("ERROR: Unknown node for Strategic Plugin: " + str(self.plugin_name))
            self.node_name = "Unknown_Node_Name"
        
        # Flag to indicate whether a plugin_discovery message has been processed for this plugin
        self.has_had_plugin_discovery_message_validated = False

        # Objects to contain the results of 'required' and 'optional' validation checks for the plugin
        self.requirement_results = RequiredStrategicComponents(self.plugin_name)
        self.optional_results = OptionalStrategicComponents()
    
    def write_strategic_final_results_to_logs(self):
        rospy.loginfo("Final validation results for Strategic Plugin: " + str(self.plugin_name))

        # Output results for required strategic plugin components
        rospy.loginfo("Required Components: ")

        if self.requirement_results.has_node:
            rospy.loginfo("Success: " + str(self.plugin_name) + " Has launched node " + str(self.node_name))
        else:
            rospy.logerr("ERROR: " + str(self.plugin_name) + " Does not have launched node " + str(self.node_name))

        if self.requirement_results.has_plan_maneuvers_service:
            rospy.loginfo("Success: " + str(self.plugin_name) + " Has advertised service " + str(self.requirement_results.plan_maneuvers_service))
        else:
            rospy.logerr("ERROR: " + str(self.plugin_name) + " Does not have advertised service " + str(self.requirement_results.plan_maneuvers_service))
        
        if self.requirement_results.has_plugin_discovery_pub:
            rospy.loginfo("Success: " + str(self.plugin_name) + " Publishes to topic " + str(self.requirement_results.plugin_discovery_topic))
        else:
            rospy.logerr("ERROR: " + str(self.plugin_name) + " Does not publish to topic " + str(self.requirement_results.plugin_discovery_topic))

        if self.requirement_results.has_correct_plugin_discovery_type:
            rospy.loginfo("Success: " + str(self.plugin_name) + " Has correct plugin discovery 'type' " +str(self.requirement_results.correct_plugin_discovery_type))
        else:
            rospy.logerr("ERROR: " + str(self.plugin_name) + " Has incorrect plugin discovery 'type'. Expected " + str(self.requirement_results.correct_plugin_discovery_type))

        if self.requirement_results.has_correct_plugin_discovery_capability:
            rospy.loginfo("Success: " + str(self.plugin_name) + " Has correct plugin discovery 'capability' " + str(self.requirement_results.correct_plugin_discovery_capability))
        else:
            rospy.logerr("ERROR: " + str(self.plugin_name) + " Has incorrect plugin discovery 'capability' " + str(self.requirement_results.correct_plugin_discovery_capability))

        # Output results for optional plugin components
        rospy.loginfo("Optional Components: ")

        if self.optional_results.has_current_pose_sub:
            rospy.loginfo("Success: " + str(self.plugin_name) + " Subscribes to " + str(self.optional_results.current_pose_topic))
        else:
            rospy.logwarn("WARNING: " + str(self.plugin_name) + " Does not subscribe to " + str(self.optional_results.current_pose_topic))
        
        if self.optional_results.has_current_speed_sub:
            rospy.loginfo("Success: " + str(self.plugin_name) + " Subscribes to " + str(self.optional_results.current_speed_topic))
        else:
            rospy.logwarn("WARNING: " + str(self.plugin_name) + " Does not subscribe to " + str(self.optional_results.current_speed_topic))
        
        if self.optional_results.has_plugin_discovery_available:
            rospy.loginfo("Success: " + str(self.plugin_name) + " Has plugin discovery message 'available' set to True")
        else:
            rospy.logwarn("WARNING: " + str(self.plugin_name) + " Does not have plugin discovery message 'available' field set to True.")

        if self.optional_results.has_plugin_discovery_activated:
            rospy.loginfo("Success: " + str(self.plugin_name) + " Has plugin discovery message 'activated' field set to True")
        else:
            rospy.logwarn("WARNING: " + str(self.plugin_name) + " Does not have plugin discovery message 'activated' field set to True.")

class RequiredStrategicComponents():
    """
    Convenience class used to store the validation results for components of a strategic plugin
    designated as required.
    """

    def __init__(self, plugin_name):
        """Constructor for RequiredStrategicComponents"""

        # Validation result indicating whether strategic plugin's node successfully launches
        self.has_node = False

        # Validation result indicating whether strategic plugin's node advertises required service
        self.plan_maneuvers_service = "/guidance/plugins/" + str(plugin_name) + "/plan_maneuvers"
        self.has_plan_maneuvers_service = False

        # Validation result indicating whether strategic plugin's node publishes to required topic
        self.plugin_discovery_topic = "/guidance/plugin_discovery"
        self.has_plugin_discovery_pub = False

        # Validation results indicating whether strategic plugin's node publishes required information to the plugin_discovery topic
        self.correct_plugin_discovery_type = Plugin.STRATEGIC
        self.has_correct_plugin_discovery_type = False

        self.correct_plugin_discovery_capability = "strategic_plan/plan_maneuvers"
        self.has_correct_plugin_discovery_capability = False

class OptionalStrategicComponents():
    """
    Convenience class used to store the validation results for components of a strategic plugin
    designated as optional.
    """

    def __init__(self):
        """Default constructor for RequiredStrategicComponents"""

        # Validation results indicating whether strategic plugin's node subscribes to optional (but commonly useful) topic
        self.current_pose_topic = "/localization/current_pose"
        self.has_current_pose_sub = False

        self.current_speed_topic = "/hardware_interface/vehicle/twist"
        self.has_current_speed_sub = False

        # Validation results indicating whether strategic plugin's node publishes optional (but commonly useful) information to the plugin_discovery topic
        self.has_plugin_discovery_available = False
        self.has_plugin_discovery_activated = False

class TacticalPluginResults:
    """
    Convenience class used to store the validation results (both required and optional) 
    for a Guidance Tactical Plugin. 
    """

    def __init__(self, plugin_name):
        """Constructor for TacticalPluginResults"""

        # Set the plugin's name, as used by the plugin_discovery topics and the plugin's advertised service(s)
        self.plugin_name = plugin_name
        
        # Set the plugin's node name
        if plugin_name == "InLaneCruisingPlugin":
            self.node_name = "/guidance/inlanecruising_plugin"
        elif plugin_name == "StopandWaitPlugin":
            self.node_name = "/guidance/stop_and_wait_plugin"
        elif plugin_name == "CooperativeLaneChangePlugin":
            self.node_name = "/guidance/cooperative_lanechange"
        elif plugin_name == "UnobstructedLaneChangePlugin":
            self.node_name = "/guidance/unobstructed_lanechange"
        elif plugin_name == "YieldPlugin":
            self.node_name = "/guidance/yield_plugin"
        else:
            rospy.logerr("ERROR: Unknown node for Tactical Plugin: " + str(self.plugin_name))
            self.node_name = "Unknown_Node_Name"
        
        # Flag to indicate whether a plugin_discovery message has been processed for this plugin
        self.has_had_plugin_discovery_message_validated = False

        # Objects to contain the results of 'required' and 'optional' validation checks for the plugin
        self.requirement_results = RequiredTacticalComponents(self.plugin_name)
        self.optional_results = OptionalTacticalComponents()

    def write_tactical_final_results_to_logs(self):
        rospy.loginfo("Final validation results for Tactical Plugin: " + str(self.plugin_name))

        # Output results for required strategic plugin components
        rospy.loginfo("Required Components: ")

        if self.requirement_results.has_node:
            rospy.loginfo("Success: " + str(self.plugin_name) + " Has launched node " + str(self.node_name))
        else:
            rospy.logerr("ERROR: " + str(self.plugin_name) + " Does not have launched node " + str(self.node_name))

        if self.requirement_results.has_plan_trajectory_service:
            rospy.loginfo("Success: " + str(self.plugin_name) + " Has advertised service " + str(self.requirement_results.plan_trajectory_service))
        else:
            rospy.logerr("ERROR: " + str(self.plugin_name) + " Does not have advertised service " + str(self.requirement_results.plan_trajectory_service))
        
        if self.requirement_results.has_plugin_discovery_pub:
            rospy.loginfo("Success: " + str(self.plugin_name) + " Publishes to topic " + str(self.requirement_results.plugin_discovery_topic))
        else:
            rospy.logerr("ERROR: " + str(self.plugin_name) + " Does not publish to topic " + str(self.requirement_results.plugin_discovery_topic))

        if self.requirement_results.has_correct_plugin_discovery_type:
            rospy.loginfo("Success: " + str(self.plugin_name) + " Has correct plugin discovery 'type' " +str(self.requirement_results.correct_plugin_discovery_type))
        else:
            rospy.logerr("ERROR: " + str(self.plugin_name) + " Has incorrect plugin discovery 'type'. Expected " + str(self.requirement_results.correct_plugin_discovery_type))

        if self.requirement_results.has_correct_plugin_discovery_capability:
            rospy.loginfo("Success: " + str(self.plugin_name) + " Has correct plugin discovery 'capability' " + str(self.requirement_results.correct_plugin_discovery_capability))
        else:
            rospy.logerr("ERROR: " + str(self.plugin_name) + " Has incorrect plugin discovery 'capability' " + str(self.requirement_results.correct_plugin_discovery_capability))

        # Output results for optional plugin components
        rospy.loginfo("Optional Components: ")

        if self.optional_results.has_current_pose_sub:
            rospy.loginfo("Success: " + str(self.plugin_name) + " Subscribes to " + str(self.optional_results.current_pose_topic))
        else:
            rospy.logwarn("WARNING: " + str(self.plugin_name) + " Does not subscribe to " + str(self.optional_results.current_pose_topic))
        
        if self.optional_results.has_current_speed_sub:
            rospy.loginfo("Success: " + str(self.plugin_name) + " Subscribes to " + str(self.optional_results.current_speed_topic))
        else:
            rospy.logwarn("WARNING: " + str(self.plugin_name) + " Does not subscribe to " + str(self.optional_results.current_speed_topic))
        
        if self.optional_results.has_plugin_discovery_available:
            rospy.loginfo("Success: " + str(self.plugin_name) + " Has plugin discovery message 'available' set to True")
        else:
            rospy.logwarn("WARNING: " + str(self.plugin_name) + " Does not have plugin discovery message 'available' field set to True.")

        if self.optional_results.has_plugin_discovery_activated:
            rospy.loginfo("Success: " + str(self.plugin_name) + " Has plugin discovery message 'activated' field set to True")
        else:
            rospy.logwarn("WARNING: " + str(self.plugin_name) + " Does not have plugin discovery message 'activated' field set to True.")

class RequiredTacticalComponents():
    """
    Convenience class used to store the validation results for components of a tactical plugin
    designated as required.
    """

    def __init__(self, plugin_name):
        """Constructor for RequiredTacticalComponents"""

        # Validation result indicating whether tactical plugin's node successfully launches
        self.has_node = False

        # Validation result indicating whether tactical plugin's node advertises required service
        self.plan_trajectory_service = "/guidance/plugins/" + str(plugin_name) + "/plan_trajectory"
        self.has_plan_trajectory_service = False

        # Validation result indicating whether tactical plugin's node publishes to required topic
        self.plugin_discovery_topic = "/guidance/plugin_discovery"
        self.has_plugin_discovery_pub = False

        # Validation results indicating whether tactical plugin's node publishes required information to the plugin_discovery topic
        self.correct_plugin_discovery_type = Plugin.TACTICAL
        self.has_correct_plugin_discovery_type = False

        self.correct_plugin_discovery_capability = "tactical_plan/plan_trajectory"
        self.has_correct_plugin_discovery_capability = False

class OptionalTacticalComponents():
    """
    Convenience class used to store the validation results for components of a strategic plugin
    designated as optional.
    """

    def __init__(self):
        """Default constructor for OptionalTacticalComponents"""

        # Validation results indicating whether tactical plugin's node subscribes to optional (but commonly useful) topic
        self.current_pose_topic = "/localization/current_pose"
        self.has_current_pose_sub = False

        self.current_speed_topic = "/hardware_interface/vehicle/twist"
        self.has_current_speed_sub = False

        # Validation results indicating whether tactical plugin's node publishes optional (but commonly useful) information to the plugin_discovery topic
        self.has_plugin_discovery_available = False
        self.has_plugin_discovery_activated = False

class ControlPluginResults:
    """
    Convenience class used to store the validation results (both required and optional) 
    for a Guidance Control Plugin. 
    """

    def __init__(self, plugin_name):
        """Constructor for ControlPluginResults"""

        # Set the plugin's name, as used by the plugin_discovery topic
        self.plugin_name = plugin_name
        
        # Set the plugin's node name and plugin_discovery capability
        if plugin_name == "Pure Pursuit":
            self.node_name = "/guidance/pure_pursuit_wrapper_node"
            self.capability = "control_pure_pursuit_plan/plan_controls"
            self.plan_trajectory_topic = "/guidance/pure_pursuit/plan_trajectory"
        else:
            rospy.logerr("ERROR: Unknown node for Control Plugin: " + str(self.plugin_name))
            self.node_name = "Unknown_Node_Name"
            self.capability = "Unknown_Capability"
            self.plan_trajectory_topic = "Unknown_Plan_Trajectory_Topic"
        
        # Flag to indicate whether a plugin_discovery message has been processed for this plugin
        self.has_had_plugin_discovery_message_validated = False

        # Objects to contain the results of 'required' and 'optional' validation checks for the plugin
        self.requirement_results = RequiredControlComponents(self.capability, self.plan_trajectory_topic)
        self.optional_results = OptionalControlComponents()
    
    def write_control_final_results_to_logs(self):
        rospy.loginfo("Final validation results for Control Plugin: " + str(self.plugin_name))

        # Output results for required strategic plugin components
        rospy.loginfo("Required Components: ")

        if self.requirement_results.has_node:
            rospy.loginfo("Success: " + str(self.plugin_name) + " Has launched node " + str(self.node_name))
        else:
            rospy.logerr("ERROR: " + str(self.plugin_name) + " Does not have launched node " + str(self.node_name))

        if self.requirement_results.has_plan_trajectory_sub:
            rospy.loginfo("Success: " + str(self.plugin_name) + " Subscribes to topic " + str(self.requirement_results.plan_trajectory_topic))
        else:
            rospy.logerr("ERROR: " + str(self.plugin_name) + " Does not subscribe to topic " + str(self.requirement_results.plan_trajectory_topic))

        if self.requirement_results.has_final_waypoints_pub:
            rospy.loginfo("Success: " + str(self.plugin_name) + " Publishes to topic " + str(self.requirement_results.final_waypoints_topic))
        else:
            rospy.logerr("ERROR: " + str(self.plugin_name) + " Does not publish to topic " + str(self.requirement_results.final_waypoints_topic))

        if self.requirement_results.has_plugin_discovery_pub:
            rospy.loginfo("Success: " + str(self.plugin_name) + " Publishes to topic " + str(self.requirement_results.plugin_discovery_topic))
        else:
            rospy.logerr("ERROR: " + str(self.plugin_name) + " Does not publish to topic " + str(self.requirement_results.plugin_discovery_topic))

        if self.requirement_results.has_correct_plugin_discovery_type:
            rospy.loginfo("Success: " + str(self.plugin_name) + " Has correct plugin discovery 'type' " +str(self.requirement_results.correct_plugin_discovery_type))
        else:
            rospy.logerr("ERROR: " + str(self.plugin_name) + " Has incorrect plugin discovery 'type'. Expected " + str(self.requirement_results.correct_plugin_discovery_type))

        if self.requirement_results.has_correct_plugin_discovery_capability:
            rospy.loginfo("Success: " + str(self.plugin_name) + " Has correct plugin discovery 'capability' " + str(self.requirement_results.correct_plugin_discovery_capability))
        else:
            rospy.logerr("ERROR: " + str(self.plugin_name) + " Has incorrect plugin discovery 'capability' " + str(self.requirement_results.correct_plugin_discovery_capability))

        # Output results for optional plugin components
        rospy.loginfo("Optional Components: ")

        if self.optional_results.has_plugin_discovery_available:
            rospy.loginfo("Success: " + str(self.plugin_name) + " Has plugin discovery message 'available' set to True")
        else:
            rospy.logwarn("WARNING: " + str(self.plugin_name) + " Does not have plugin discovery message 'available' field set to True.")

        if self.optional_results.has_plugin_discovery_activated:
            rospy.loginfo("Success: " + str(self.plugin_name) + " Has plugin discovery message 'activated' field set to True")
        else:
            rospy.logwarn("WARNING: " + str(self.plugin_name) + " Does not have plugin discovery message 'activated' field set to True.")

class RequiredControlComponents():
    """
    Convenience class used to store the validation results for components of a control plugin
    designated as required.
    """

    def __init__(self, plugin_capability, plan_trajectory_topic):
        """Constructor for RequiredControlComponents"""

        # Validation result indicating whether control plugin's node successfully launches
        self.has_node = False

        # Validation result indicating whether control plugin's node subscribes to required topics
        self.plan_trajectory_topic = plan_trajectory_topic
        self.has_plan_trajectory_sub = False

        # Validation result indicating whether control plugin's node publishes to required topics
        self.plugin_discovery_topic = "/guidance/plugin_discovery"
        self.has_plugin_discovery_pub = False

        self.final_waypoints_topic = "/guidance/carma_final_waypoints"
        self.has_final_waypoints_pub = False

        # Validation results indicating whether control plugin's node publishes required information to the plugin_discovery topic
        self.correct_plugin_discovery_type = Plugin.CONTROL
        self.has_correct_plugin_discovery_type = False

        self.correct_plugin_discovery_capability = plugin_capability
        self.has_correct_plugin_discovery_capability = False

class OptionalControlComponents():
    """
    Convenience class used to store the validation results for components of a control plugin
    designated as required.
    """

    def __init__(self):
        """Default constructor for OptionalControlComponents"""

        # Validation results indicating whether control plugin's node publishes optional (but commonly useful) information to the plugin_discovery topic
        self.has_plugin_discovery_available = False
        self.has_plugin_discovery_activated = False