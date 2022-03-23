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
import rosnode
import guidance_plugin_components 
from cav_msgs.msg import Plugin 
from cav_msgs.msg import SystemAlert

class GuidancePluginValidator:
    """
    Primary class for the guidance_plugin_validator node. Conducts the validation of each of the
    Guidance Plugins (of type 'Strategic', 'Tactical', or 'Control-Wrapper') as provided by this node's
    configuration parameters.
    """

    def __init__(self):
        """Default constructor for GuidancePluginValidator"""

        # Create plugin_discovery subscriber
        self.plugin_discovery_sub = rospy.Subscriber("plugin_discovery", Plugin, self.plugin_discovery_cb)
        self.system_alert_sub = rospy.Subscriber("system_alert", SystemAlert, self.system_alert_cb)

        # Read in config params
        self.validation_duration = rospy.get_param('~validation_duration', 300) # Maximum time (sec) that node will spend conducting validation before results are considered final
        self.strategic_plugin_names = rospy.get_param('~strategic_plugins_to_validate', [])
        self.tactical_plugin_names = rospy.get_param('~tactical_plugins_to_validate', [])
        self.control_plugin_names = rospy.get_param('~control_plugins_to_validate', []) 

        # Write config params to log file
        rospy.loginfo("Config params for guidance_plugin_validator:")
        rospy.loginfo("Validation Duration: " + str(self.validation_duration) + " seconds")
        rospy.loginfo("Strategic Guidance Plugins: " + str(self.strategic_plugin_names))
        rospy.loginfo("Tactical Guidance Plugins: " + str(self.tactical_plugin_names))
        rospy.loginfo("Control Guidance Plugins: " + str(self.control_plugin_names))

        # Boolean flag to indicate whether drivers are ready (this indicates that plugin node validation checks can begin)
        self.has_startup_completed = False

        # Boolean flag to indicate whether each guidance plugin's node has been validated
        self.has_node_validation_completed = False

        # Boolean flag to indicate whether final results have been written to log file
        self.has_logged_final_results = False

        # Set spin rate
        self.spin_rate = rospy.Rate(10) # 10 Hz
        
        # Initialize empty dicts that will be populated with a <plugin-type>PluginResults object for each Guidance Plugin
        self.strategic_plugin_validation_results = {} # Key is plugin's name; Value is plugin's StrategicPluginResults object
        self.tactical_plugin_validation_results = {}  # Key is plugin's name; Value is plugin's TacticalPluginResults object
        self.control_plugin_validation_results = {}   # Key is plugin's name; Value is plugin's ControlPluginResults object

        # Call member function to populate the 'validation results' dicts
        self.populate_results_dicts(self.strategic_plugin_names, self.tactical_plugin_names, self.control_plugin_names)

    def populate_results_dicts(self, strategic_plugin_names, tactical_plugin_names, control_plugin_names):
        """Initialize the the 'validation results' lists and 'index by name' dicts for this object"""

        # Populate validation results dict for Strategic Plugins
        for plugin_name in strategic_plugin_names:
            self.strategic_plugin_validation_results[plugin_name] = guidance_plugin_components.StrategicPluginResults(plugin_name)

        # Populate validation results dict for Tactical Plugins
        for plugin_name in tactical_plugin_names:
            self.tactical_plugin_validation_results[plugin_name] = guidance_plugin_components.TacticalPluginResults(plugin_name)
        
        # Populate validation results dict for Control Plugins
        for plugin_name in control_plugin_names:
            self.control_plugin_validation_results[plugin_name] = guidance_plugin_components.ControlPluginResults(plugin_name)

        return

    def spin(self):
        """
        Function to ensure node spins at configured spin rate.
        """

        while not rospy.is_shutdown():
            if self.has_startup_completed:
                
                # Conduct node validation if it has not yet occurred
                if not self.has_node_validation_completed:
                    self.conduct_node_validation()
                    self.has_node_validation_completed = True

                # If time has surpassed the configured validation duration, the current results are considered final. Write to log file.
                seconds_since_startup_completed = rospy.get_time() - self.start_time_seconds
                if (seconds_since_startup_completed >= self.validation_duration):
                    if not self.has_logged_final_results:
                        self.log_final_results_for_each_plugin()
                        self.has_logged_final_results = True
                     
            self.spin_rate.sleep()

        return
    
    def log_final_results_for_each_plugin(self):
        """
        Calls appropriate function for each plugin's 'results' object in order to write
        all final validation results to the log file for this node. 
        """

        rospy.loginfo("**********************************************************")
        rospy.loginfo("******Final Validation Results for Strategic Plugins******")
        rospy.loginfo("**********************************************************")
        # Write final validation results to log file for Guidance Strategic Plugins
        for plugin_name, plugin_results_object in self.strategic_plugin_validation_results.items():
            plugin_results_object.write_strategic_final_results_to_logs()
        
        rospy.loginfo("**********************************************************")
        rospy.loginfo("******Final Validation Results for Tactical Plugins*******")
        rospy.loginfo("**********************************************************")
        # Write final validation results to log file for Guidance Tactical Plugins
        for plugin_name, plugin_results_object in self.tactical_plugin_validation_results.items():
            plugin_results_object.write_tactical_final_results_to_logs()

        rospy.loginfo("**********************************************************")
        rospy.loginfo("*******Final Validation Results for Control Plugins*******")
        rospy.loginfo("**********************************************************")
        # Write final validation results to log file for Guidance Control Plugins
        for plugin_name, plugin_results_object in self.control_plugin_validation_results.items():
            plugin_results_object.write_control_final_results_to_logs()

        rospy.loginfo("**********************************************************")
        rospy.loginfo("*******End of Final Validation Results for Plugins********")
        rospy.loginfo("**********************************************************")

        return

    def system_alert_cb(self, msg):
        """
        Callback function for the system_alert topic. The Guidance Plugin Validator Node will begin conducting
        validation checks on each plugin's node after a 'DRIVERS_READY' alert has been received.
        """

        # Startup has completed when drivers are ready
        if msg.type == SystemAlert.DRIVERS_READY:
            rospy.loginfo("DRIVERS_READY message received. Beginning node validation.")
            self.start_time_seconds = rospy.get_time()
            self.has_startup_completed = True

        return
    
    def plugin_discovery_cb(self, msg):
        """
        Callback function for the plugin_discovery topic. Processes the first received message for each guidance
        plugin (as specified by this node's configuration parameters), and updates the plugin's 'results' 
        object accordingly.
        """

        # Get the name of this plugin based on the message
        plugin_name = msg.name
        
        # Validate the plugin_discovery message based on the plugin's type (Strategic, Tactical, or Control)
        if plugin_name in self.strategic_plugin_names:

            # Do not process message if this plugin has already had its plugin_discovery message validated
            # Note: Assumption is that once one message is received and processed for a plugin, the rest will be identical
            if self.strategic_plugin_validation_results[plugin_name].has_had_plugin_discovery_message_validated:
                return

            # Process the message and log appropriate messages
            rospy.loginfo("Processing plugin_discovery message for " + str(plugin_name) + " (Strategic Plugin)")
            self.strategic_plugin_validation_results[plugin_name].has_had_plugin_discovery_message_validated = True

            expected_capability = self.strategic_plugin_validation_results[plugin_name].requirement_results.correct_plugin_discovery_capability
            if msg.capability == expected_capability:
                self.strategic_plugin_validation_results[plugin_name].requirement_results.has_correct_plugin_discovery_capability = True
                rospy.loginfo("Success: " + str(plugin_name) + " has plugin_discovery capability " + str(expected_capability))
            else:
                rospy.logerr("ERROR: " + str(plugin_name) + " plugin_discovery capability == " + str(msg.capability) + " (expected + " + str(expected_capability) + ")")

            expected_type = self.strategic_plugin_validation_results[plugin_name].requirement_results.correct_plugin_discovery_type
            if msg.type == expected_type:
                self.strategic_plugin_validation_results[plugin_name].requirement_results.has_correct_plugin_discovery_type = True
                rospy.loginfo("Success: " + str(plugin_name) + " has plugin_discovery type " + str(expected_type))
            else:
                rospy.logerr("ERROR: " + str(plugin_name) + " plugin_discovery type == " + str(msg.type) + " (expected + " + str(expected_type) + ")")

            if msg.available == True:
                self.strategic_plugin_validation_results[plugin_name].optional_results.has_correct_plugin_discovery_available = True
                rospy.loginfo("Success: " + str(plugin_name) + " has plugin_discovery availabile == True")
            else:
                rospy.logwarn("WARNING: " + str(plugin_name) + " plugin_discovery available == " + str(msg.available) + " (expected True)")

            if msg.activated == True:
                self.strategic_plugin_validation_results[plugin_name].optional_results.has_correct_plugin_discovery_activated = True
                rospy.loginfo("Success: " + str(plugin_name) + " has plugin_discovery activated == True")
            else:
                rospy.logwarn("WARNING: " + str(plugin_name) + " plugin_discovery activated == " + str(msg.activated) + " (expected True)")

        elif plugin_name in self.tactical_plugin_names:

            # Do not process message if this plugin has already had its plugin_discovery message validated
            # Note: Assumption is that once one message is received and processed for a plugin, the rest will be identical
            if self.tactical_plugin_validation_results[plugin_name].has_had_plugin_discovery_message_validated:
                return

            # Process the message and log appropriate messages
            rospy.loginfo("Processing plugin_discovery message for " + str(plugin_name) + " (Tactical Plugin)")
            self.tactical_plugin_validation_results[plugin_name].has_had_plugin_discovery_message_validated = True

            expected_capability = self.tactical_plugin_validation_results[plugin_name].requirement_results.correct_plugin_discovery_capability 
            if msg.capability == expected_capability:
                self.tactical_plugin_validation_results[plugin_name].requirement_results.has_correct_plugin_discovery_capability = True
                rospy.loginfo("Success: " + str(plugin_name) + " has plugin_discovery capability " + str(expected_capability))
            else:
                rospy.logerr("ERROR: " + str(plugin_name) + " plugin_discovery capability == " + str(msg.capability) + " (expected + " + str(expected_capability) + ")")

            expected_type = self.tactical_plugin_validation_results[plugin_name].requirement_results.correct_plugin_discovery_type
            if msg.type == expected_type:
                self.tactical_plugin_validation_results[plugin_name].requirement_results.has_correct_plugin_discovery_type = True
                rospy.loginfo("Success: " + str(plugin_name) + " has plugin_discovery type " + str(expected_type))
            else:
                rospy.logerr("ERROR: " + str(plugin_name) + " plugin_discovery type == " + str(msg.type) + " (expected + " + str(expected_type) + ")")

            if msg.available == True:
                self.tactical_plugin_validation_results[plugin_name].optional_results.has_correct_plugin_discovery_available = True
                rospy.loginfo("Success: " + str(plugin_name) + " has plugin_discovery availabile == True")
            else:
                rospy.logwarn("WARNING: " + str(plugin_name) + " plugin_discovery availability == " + str(msg.available) + " (expected True)")

            if msg.activated == True:
                self.tactical_plugin_validation_results[plugin_name].optional_results.has_correct_plugin_discovery_activated = True
                rospy.loginfo("Success: " + str(plugin_name) + " has plugin_discovery activated == True")
            else:
                rospy.logwarn("WARNING: " + str(plugin_name) + " plugin_discovery activated == " + str(msg.activated) + " (expected True)")

        elif plugin_name in self.control_plugin_names:

            # Do not process message if this plugin has already had its plugin_discovery message validated
            # Note: Assumption is that once one message is received and processed for a plugin, the rest will be identical
            if self.control_plugin_validation_results[plugin_name].has_had_plugin_discovery_message_validated:
                return

            # Process the message and log appropriate messages
            rospy.loginfo("Processing plugin_discovery message for " + str(plugin_name) + " (Control Plugin)")
            self.control_plugin_validation_results[plugin_name].has_had_plugin_discovery_message_validated = True

            expected_capability = self.control_plugin_validation_results[plugin_name].requirement_results.correct_plugin_discovery_capability 
            if msg.capability == expected_capability:
                self.control_plugin_validation_results[plugin_name].requirement_results.has_correct_plugin_discovery_capability = True
                rospy.loginfo("Success: " + str(plugin_name) + " has plugin_discovery capability " + str(expected_capability))
            else:
                rospy.logerr("ERROR: " + str(plugin_name) + " plugin_discovery capability == " + str(msg.capability) + " (expected + " + str(expected_capability) + ")")

            expected_type = self.control_plugin_validation_results[plugin_name].requirement_results.correct_plugin_discovery_type
            if msg.type == expected_type:
                self.control_plugin_validation_results[plugin_name].requirement_results.has_correct_plugin_discovery_type = True
                rospy.loginfo("Success: " + str(plugin_name) + " has plugin_discovery type " + str(expected_type))
            else:
                rospy.logerr("ERROR: " + str(plugin_name) + " plugin_discovery type == " + str(msg.type) + " (expected + " + str(expected_type) + ")")

            if msg.available == True:
                self.control_plugin_validation_results[plugin_name].optional_results.has_correct_plugin_discovery_available = True
                rospy.loginfo("Success: " + str(plugin_name) + " has plugin_discovery availabile == True")  
            else:
                rospy.logwarn("WARNING: " + str(plugin_name) + " plugin_discovery availability == " + str(msg.available) + " (expected True)")

            if msg.activated == True:
                self.control_plugin_validation_results[plugin_name].optional_results.has_correct_plugin_discovery_activated = True
                rospy.loginfo("Success: " + str(plugin_name) + " has plugin_discovery activated == True")
            else:
                rospy.logwarn("WARNING: " + str(plugin_name) + " plugin_discovery activated == " + str(msg.activated) + " (expected True)")

        return

    def conduct_node_validation(self):
        """
        Call appropriate member functions to conduct validation of node communication interfaces.
        """

        rospy.loginfo("Beginning validation checks for node subscriptions, publications, and advertised services")

        self.validate_strategic_plugins()
        self.validate_tactical_plugins()
        self.validate_control_plugins()

        rospy.loginfo("Completed validation checks for node subscriptions, publications, and advertised services")

        return

    def validate_strategic_plugins(self):
        """
        Conduct validation checks for each strategic plugin's node (as specified by this node's 
        configuration parameters) for proper publications, subscriptions, and advertised services. Based on the
        results, this function updates each strategic plugin's StrategicPluginResults object accordingly.
        """
        for plugin_name, plugin_results_object in self.strategic_plugin_validation_results.items():
            plugin_node_name = plugin_results_object.node_name
            rospy.loginfo("Processing publishers, subscribers, and services for " + str(plugin_name) + " (Strategic Plugin)")

            # Check whether the node has been created
            if rosnode.rosnode_ping(plugin_node_name, max_count = 5):
                plugin_results_object.requirement_results.has_node = True
                rospy.loginfo("Success: Node " + str(plugin_node_name) + " exists.")
            else:
                rospy.logerr("ERROR: No node response for " + str(plugin_node_name) + ". Node does not exist.")
            
            # Obtain string that includes information regarding a node's publications, subscriptions, and services
            rosnode_info_string = (rosnode.get_node_info_description(plugin_node_name))

            # Get substring from rosnode info that contains 'Subscriptions' information
            sub_index_start = rosnode_info_string.index("Subscriptions:")
            sub_index_end = rosnode_info_string.index("Services:")
            subscriptions_string = rosnode_info_string[sub_index_start:sub_index_end]

            # Check for required and optional subscriptions
            if plugin_results_object.optional_results.current_pose_topic in subscriptions_string:
                plugin_results_object.optional_results.has_current_pose_sub = True
                rospy.loginfo("Success: " + str(plugin_node_name) + " subscribes to " + str(plugin_results_object.optional_results.current_pose_topic))
            else:
                rospy.logwarn("WARNING: " + str(plugin_node_name) + " does not subscribe to " + str(plugin_results_object.optional_results.current_pose_topic))
            
            if plugin_results_object.optional_results.current_speed_topic in subscriptions_string:
                plugin_results_object.optional_results.has_current_speed_sub = True
                rospy.loginfo("Success: " + str(plugin_node_name) + " subscribes to " + str(plugin_results_object.optional_results.current_speed_topic))
            else:
                rospy.logwarn("WARNING: " + str(plugin_node_name) + " does not subscribe to " + str(plugin_results_object.optional_results.current_speed_topic))

            # Get substring from rosnode info that contains 'Publications' information
            pub_index_start = rosnode_info_string.index("Publications:")
            pub_index_end = rosnode_info_string.index("Subscriptions:")
            publications_string = rosnode_info_string[pub_index_start:pub_index_end]

            # Check for required and optional publications
            if plugin_results_object.requirement_results.plugin_discovery_topic in publications_string:
                plugin_results_object.requirement_results.has_plugin_discovery_pub = True
                rospy.loginfo("Success: " + str(plugin_node_name) + " publishes to " + str(plugin_results_object.requirement_results.plugin_discovery_topic))
            else:
                rospy.logerr("ERROR: " + str(plugin_node_name) + " does not publish to " + str(plugin_results_object.requirement_results.plugin_discovery_topic))

            # Get substring from rosnode info that contains 'Services' information
            serv_index_start = rosnode_info_string.index("Services:")
            services_string = rosnode_info_string[serv_index_start:]

            # Check for required and optional servers
            if plugin_results_object.requirement_results.plan_maneuvers_service in services_string:
                plugin_results_object.requirement_results.has_plan_maneuvers_service = True
                rospy.loginfo("Success: " + str(plugin_node_name) + " advertises service " + str(plugin_results_object.requirement_results.plan_maneuvers_service))
            else:
                rospy.logerr("ERROR: " + str(plugin_node_name) + " does not advertise service " + str(plugin_results_object.requirement_results.plan_maneuvers_service))

        return

    def validate_tactical_plugins(self):
        """
        Conduct validation checks for each tactical plugin's node (as specified by this node's 
        configuration parameters) for proper publications, subscriptions, and advertised services. Based on the
        results, this function updates each tactical plugin's TacticalPluginResults object accordingly.
        """
        for plugin_name, plugin_results_object in self.tactical_plugin_validation_results.items():
            plugin_node_name = plugin_results_object.node_name
            rospy.loginfo("Processing publishers, subscribers, and services for " + str(plugin_name) + " (Tactical Plugin)")

            # Check whether the node has been created
            if rosnode.rosnode_ping(plugin_node_name, max_count = 5):
                plugin_results_object.requirement_results.has_node = True
                rospy.loginfo("Success: Node " + str(plugin_node_name) + " exists.")
            else:
                rospy.logerr("ERROR: No node response for " + str(plugin_node_name) + ". Node does not exist.")
            
            # Obtain string that includes information regarding a node's publications, subscriptions, and services
            rosnode_info_string = (rosnode.get_node_info_description(plugin_node_name))

            # Get substring from rosnode info that contains 'Subscriptions' information
            sub_index_start = rosnode_info_string.index("Subscriptions:")
            sub_index_end = rosnode_info_string.index("Services:")
            subscriptions_string = rosnode_info_string[sub_index_start:sub_index_end]

            # Check for required and optional subscriptions
            if plugin_results_object.optional_results.current_pose_topic in subscriptions_string:
                plugin_results_object.optional_results.has_current_pose_sub = True
                rospy.loginfo("Success: " + str(plugin_node_name) + " subscribes to " + str(plugin_results_object.optional_results.current_pose_topic))
            else:
                rospy.logwarn("WARNING: " + str(plugin_node_name) + " does not subscribe to " + str(plugin_results_object.optional_results.current_pose_topic))
            
            if plugin_results_object.optional_results.current_speed_topic in subscriptions_string:
                plugin_results_object.optional_results.has_current_speed_sub = True
                rospy.loginfo("Success: " + str(plugin_node_name) + " subscribes to " + str(plugin_results_object.optional_results.current_speed_topic))
            else:
                rospy.logwarn("WARNING: " + str(plugin_node_name) + " does not subscribe to " + str(plugin_results_object.optional_results.current_speed_topic))

            # Get substring from rosnode info that contains 'Publications' information
            pub_index_start = rosnode_info_string.index("Publications:")
            pub_index_end = rosnode_info_string.index("Subscriptions:")
            publications_string = rosnode_info_string[pub_index_start:pub_index_end]

            # Check for required and optional publications
            if plugin_results_object.requirement_results.plugin_discovery_topic in publications_string:
                plugin_results_object.requirement_results.has_plugin_discovery_pub = True
                rospy.loginfo("Success: " + str(plugin_node_name) + " publishes to " + str(plugin_results_object.requirement_results.plugin_discovery_topic))
            else:
                rospy.logerr("ERROR: " + str(plugin_node_name) + " does not publish to " + str(plugin_results_object.requirement_results.plugin_discovery_topic))

            # Get substring from rosnode info that contains 'Services' information
            serv_index_start = rosnode_info_string.index("Services:")
            services_string = rosnode_info_string[serv_index_start:]

            # Check for required and optional servers
            if plugin_results_object.requirement_results.plan_trajectory_service in services_string:
                plugin_results_object.requirement_results.has_plan_trajectory_service = True
                rospy.loginfo("Success: " + str(plugin_node_name) + " advertises service " + str(plugin_results_object.requirement_results.plan_trajectory_service))
            else:
                rospy.logerr("ERROR: " + str(plugin_node_name) + " does not advertise service " + str(plugin_results_object.requirement_results.plan_trajectory_service))

        return

    def validate_control_plugins(self):
        """
        Conduct validation checks for each control plugin's node (as specified by this node's 
        configuration parameters) for proper publications, subscriptions, and advertised services. Based on the
        results, this function updates each control plugin's ControlPluginResults object accordingly.
        """
        for plugin_name, plugin_results_object in self.control_plugin_validation_results.items():
            plugin_node_name = plugin_results_object.node_name
            rospy.loginfo("Processing publishers, subscribers, and services for " + str(plugin_name) + " (Control Plugin)")

            # Check whether the node has been created
            if rosnode.rosnode_ping(plugin_node_name, max_count = 5):
                plugin_results_object.requirement_results.has_node = True
                rospy.loginfo("Success: Node " + str(plugin_node_name) + " exists.")
            else:
                rospy.logerr("ERROR: No node response for " + str(plugin_node_name) + ". Node does not exist.")
            
            # Obtain string that includes information regarding a node's publications, subscriptions, and services
            rosnode_info_string = (rosnode.get_node_info_description(plugin_node_name))

            # Get substring from rosnode info that contains 'Subscriptions' information
            sub_index_start = rosnode_info_string.index("Subscriptions:")
            sub_index_end = rosnode_info_string.index("Services:")
            subscriptions_string = rosnode_info_string[sub_index_start:sub_index_end]

            # Check for required and optional subscriptions
            if plugin_results_object.requirement_results.plan_trajectory_topic in subscriptions_string:
                plugin_results_object.requirement_results.has_plan_trajectory_sub = True
                rospy.loginfo("Success: " + str(plugin_node_name) + " subscribes to " + str(plugin_results_object.requirement_results.plan_trajectory_topic))
            else:
                rospy.logerr("ERROR: " + str(plugin_node_name) + " does not subscribe to " + str(plugin_results_object.requirement_results.plan_trajectory_topic))
            
            # Get substring from rosnode info that contains 'Publications' information
            pub_index_start = rosnode_info_string.index("Publications:")
            pub_index_end = rosnode_info_string.index("Subscriptions:")
            publications_string = rosnode_info_string[pub_index_start:pub_index_end]

            # Check for required and optional publications
            if plugin_results_object.requirement_results.plugin_discovery_topic in publications_string:
                plugin_results_object.requirement_results.has_plugin_discovery_pub = True
                rospy.loginfo("Success: " + str(plugin_node_name) + " publishes to " + str(plugin_results_object.requirement_results.plugin_discovery_topic))
            else:
                rospy.logerr("ERROR: " + str(plugin_node_name) + " does not publish to " + str(plugin_results_object.requirement_results.plugin_discovery_topic))

            if plugin_results_object.requirement_results.final_waypoints_topic in publications_string:
                plugin_results_object.requirement_results.has_final_waypoints_pub = True
                rospy.loginfo("Success: " + str(plugin_node_name) + " publishes to " + str(plugin_results_object.requirement_results.final_waypoints_topic))
            else:
                rospy.logerr("ERROR: " + str(plugin_node_name) + " does not publish to " + str(plugin_results_object.requirement_results.final_waypoints_topic))

        return