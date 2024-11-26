#pragma once

/*
 * Copyright (C) 2020-2022 LEIDOS.
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
 */

#include <rclcpp/rclcpp.hpp>

#include <carma_planning_msgs/msg/route.hpp>
#include <carma_planning_msgs/msg/route_state.hpp>
#include <carma_planning_msgs/msg/route_event.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <carma_planning_msgs/srv/get_available_routes.hpp>
#include <carma_planning_msgs/srv/set_active_route.hpp>
#include <carma_planning_msgs/srv/abort_active_route.hpp>

#include <carma_wm/WMListener.hpp>
#include <carma_wm/WorldModel.hpp>
#include <carma_wm/Geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <lanelet2_extension/io/autoware_osm_parser.h>
#include <lanelet2_core/utility/Utilities.h>

#include <wgs84_utils/wgs84_utils.h>
#include <boost/filesystem.hpp>
#include <boost/optional.hpp>
#include <math.h>
#include <unordered_set>
#include <functional>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "route/route_state_worker.hpp"

namespace route {

    class RouteGeneratorWorker
    {

    public:
        /**
         * \brief Constructor for RouteGeneratorWorker class taking in dependencies via dependency injection
         */
        RouteGeneratorWorker(tf2_ros::Buffer& tf2_buffer);

        /**
         * \brief reroutingChecker function to set the rerouting flag locally
         */
        std::function<bool()> reroutingChecker;

        /**
         * \brief setReroutingChecker function to set the rerouting flag
         */
        void setReroutingChecker(const std::function<bool()> inputFunction);
        
        /**
         * \brief Dependency injection for world model pointer.
         * \param wm CARMA world model object providing lanelet vector map and traffic regulations
         */
        void setWorldModelPtr(carma_wm::WorldModelConstPtr wm);

        /**
         * \brief Dependency injection for logger interface.
         * \param logger Logger interface that will be used by worker class
         */
        void setLoggerInterface(rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger);

        /**
         * \brief Dependency injection for clock object.
         * \param clock Clock object that will be used by worker class
         */
        void setClock(rclcpp::Clock::SharedPtr clock);

        /**
         * \brief Generate Lanelet2 route based on input destinations
         * \param start Lanelet 2D point in map frame indicates the starting point of selected route
         * \param via A vector of lanelet 2D points in map frame which contains points we want to visit along the route
         * \param end Lanelet 2D point in map frame indicates the final destination of selected route
         * \param map_pointer A constant pointer to lanelet vector map
         * \param graph_pointer A constant pointer to lanelet vector map routing graph
         */
        lanelet::Optional<lanelet::routing::Route> routing(const lanelet::BasicPoint2d start,
                                                        const std::vector<lanelet::BasicPoint2d>& via,
                                                        const lanelet::BasicPoint2d end,
                                                        const lanelet::LaneletMapConstPtr map_pointer,
                                                        const carma_wm::LaneletRoutingGraphConstPtr graph_pointer) const;

        /**
         * \brief Get_available_route service callback. Calls to this service will respond with a list of route names for user to select
         * \param req An empty carma_planning_msgs::srv::GetAvailableRoutes::Request
         * \param resp A carma_planning_msgs::srv::GetAvailableRoutes::Response msg contains a list of empty Route messages with only route name populated
         */
        bool getAvailableRouteCb(const std::shared_ptr<rmw_request_id_t>,
                                const std::shared_ptr<carma_planning_msgs::srv::GetAvailableRoutes::Request>,
                                std::shared_ptr<carma_planning_msgs::srv::GetAvailableRoutes::Response> resp);
        
        /**
         * \brief Set_active_route service callback. User can select a route to start following by calling this service
         * \param req A carma_planning_msgs::srv::SetActiveRoute::Request msg which contains either a route name a user wants to select or 
         *            an array of carma_v2x_msgs::msg::Position3D destination points to generate a route from.
         * \param resp A carma_planning_msgs::srv::SetActiveRoute::Response msg contains error status indicating whether the routing succeeded
         */
        bool setActiveRouteCb(const std::shared_ptr<rmw_request_id_t>,
                                const std::shared_ptr<carma_planning_msgs::srv::SetActiveRoute::Request> req,
                                std::shared_ptr<carma_planning_msgs::srv::SetActiveRoute::Response> resp);

        /**
         * \brief Abort_active_route service callback. User can call this service to abort a current following route and return to route selection stage
         * \param req A carma_planning_msgs::srv::AbortActiveRoute::Request msg which contains the route name user wants to stop following
         * \param resp A carma_planning_msgs::srv::AbortActiveRoute::Response msg contains error status showing if there is an active route
         */
        bool abortActiveRouteCb(const std::shared_ptr<rmw_request_id_t>,
                                const std::shared_ptr<carma_planning_msgs::srv::AbortActiveRoute::Request>,
                                std::shared_ptr<carma_planning_msgs::srv::AbortActiveRoute::Response> resp);

        /**
         * \brief Callback for the front bumper pose transform
         */
        void bumperPoseCb();

        /**
         * \brief Callback for the twist subscriber, which will store latest twist locally
         * \param msg Latest twist message
         */
        void twistCb(geometry_msgs::msg::TwistStamped::UniquePtr msg);

        /**
         * \brief Callback for the georeference subscriber used to set the map projection
         * \param msg The latest georeference
         */ 
        void georeferenceCb(std_msgs::msg::String::UniquePtr msg);

        /**
         * \brief Set method for configurable parameter
         * \param path The location of route files
         */
        void setRouteFilePath(const std::string& path);

        /**
         * \brief Set method for configurable parameter
         * \param dt_dest_range Minimum down track error which can trigger route complete event
         */
        void setDowntrackDestinationRange(double dt_dest_range);

        /**
         * \brief Method to pass publishers into worker class
         * \param route_event_pub Route event publisher
         * \param route_state_pub Route state publisher
         * \param route_pub Route publisher
         * \param route_marker_pub Route marker publisher
         */
        void setPublishers(const carma_ros2_utils::PubPtr<carma_planning_msgs::msg::RouteEvent>& route_event_pub,
                        const carma_ros2_utils::PubPtr<carma_planning_msgs::msg::RouteState>& route_state_pub,
                        const carma_ros2_utils::PubPtr<carma_planning_msgs::msg::Route>& route_pub,
                        const carma_ros2_utils::PubPtr<visualization_msgs::msg::Marker>& route_marker_pub);
        
        /**
         * \brief Helper function to check whether a route's shortest path contains any duplicate Lanelet IDs.
         *        'true' indicates that the route's shortest path contains duplicate Lanelet IDs.
         * \param route Route object from lanelet2 lib routing function
         */
        bool checkForDuplicateLaneletsInShortestPath(const lanelet::routing::Route& route) const;

        /**
         * \brief Function to take route destination points from a vector of 3D points and convert them from lat/lon values to to coordinates in map frame based on the projection string
         * \param destinations A vector of carma_v2x_msgs::msg::Position3D points containing destination points provided as lat/long values
         */
        std::vector<lanelet::BasicPoint3d> loadRouteDestinationsInMapFrame(const std::vector<carma_v2x_msgs::msg::Position3D>& destinations) const;

        /**
         * \brief Function to load route destination points from a route file and store them in a vector of 3D points
         * \param route_id This function will read the route file with provided route_id
         */
        std::vector<carma_v2x_msgs::msg::Position3D> loadRouteDestinationGpsPointsFromRouteId(const std::string& route_id) const;

        /**
         * \brief Helper function to generate a CARMA route message based on planned lanelet route
         * \param route Route object from lanelet2 lib routing function
         */
        carma_planning_msgs::msg::Route composeRouteMsg(const lanelet::Optional<lanelet::routing::Route>& route) const;

        /**
         * \brief Spin callback which will be called frequently based on spin rate
         */
        bool spinCallback();

        /**
         * \brief composeRouteMarkerMsg is a function to generate route rviz markers
         * \param route Route object from lanelet2 lib routing function
         */
        visualization_msgs::msg::Marker composeRouteMarkerMsg(const lanelet::Optional<lanelet::routing::Route>& route);

        /**
        * \brief crosstrackErrorCheck is a function that determines when the vehicle has left the route and reports when a crosstrack error has
        * taken place
        * 
        *  \param msg Msg that contains the vehicle's current position
        *  \param current_llt The lanelet that the vehicle is currently in
        * */
        bool crosstrackErrorCheck(const std::shared_ptr<geometry_msgs::msg::PoseStamped>& msg, lanelet::ConstLanelet current_llt);

        /**
         * \brief set the crosstrack error counter maximum limit
         * 
         *  \param cte_max the maximum amount of acceptable crosstrack error instances
        */
        void setCrosstrackErrorCountMax(int cte_max);

        /**
         * \brief set the maximum crosstrack error distance
         * 
         *  \param cte_dist maximum distance value (specified in the this package's parameters.yaml configuration file)
        */
        void setCrosstrackErrorDistance(double cte_dist);

        /**
         * \brief "Get the closest lanelet on the route relative to the vehicle's current position. 
         * If the input list does not contain lanelets on the route, still closest lanelet from the route will be returned
         * 
         *  \param position the current position of the vehicle
        */
        lanelet::ConstLanelet getClosestLaneletFromRouteLanelets(lanelet::BasicPoint2d position) const;

        // Added for Unit Testing
        void addLanelet(lanelet::ConstLanelet llt);

        /**
         * \brief After route is invalidated, this function returns a new route based on the destinations points.
         * \param destination_points_in_map vector of destination points
         * \note Destination points will be removed if the current pose is past those points.
        */
        lanelet::Optional<lanelet::routing::Route> rerouteAfterRouteInvalidation(const std::vector<lanelet::BasicPoint2d>& destination_points_in_map);

        /**
         * \brief Initialize transform lookup from front bumper to map
         */
        void initializeBumperTransformLookup();

        // Current vehicle pose if it has been recieved
        boost::optional<geometry_msgs::msg::PoseStamped> vehicle_pose_;

    private:

        const double DEG_TO_RAD = 0.0174533;
        
        // route state worker
        RouteStateWorker rs_worker_;

        // directory of route files
        std::string route_file_path_;

        // const pointer to world model object
        carma_wm::WorldModelConstPtr world_model_;

        // route messages waiting to be updated and published
        carma_planning_msgs::msg::Route route_msg_;
        carma_planning_msgs::msg::RouteEvent route_event_msg_;
        carma_planning_msgs::msg::RouteState route_state_msg_;
        visualization_msgs::msg::Marker route_marker_msg_;

        std::vector<lanelet::ConstPoint3d> points_; 
        
        //List of lanelets in the route
        lanelet::ConstLanelets route_llts;
        
        // minimum down track error which can trigger route complete event
        double down_track_target_range_;

        // current cross track distance
        double current_crosstrack_distance_;

        // current down track distance relevant to the route
        double current_downtrack_distance_;

        // current pose
        lanelet::BasicPoint2d current_loc_;

        // current lanelet cross track distance
        double ll_crosstrack_distance_;

        // current lanelet down track
        double ll_downtrack_distance_;

        lanelet::Id ll_id_;

        // current speed limit on current lanelet
        double speed_limit_ = 0;
        // Current vehicle forward speed
        double current_speed_ = 0;
        //A small static value for comparing doubles
        static constexpr double epsilon_ = 0.001;

        // local copy of Route publihsers
        carma_ros2_utils::PubPtr<carma_planning_msgs::msg::RouteEvent> route_event_pub_;
        carma_ros2_utils::PubPtr<carma_planning_msgs::msg::RouteState> route_state_pub_;
        carma_ros2_utils::PubPtr<carma_planning_msgs::msg::Route> route_pub_;
        carma_ros2_utils::PubPtr<visualization_msgs::msg::Marker> route_marker_pub_;

        // a bool flag indicates a new route has been generated such that a local copy of Route message should be published again
        bool new_route_msg_generated_ = false;
        bool new_route_marker_generated_ = false;

        // a queue of route event. All events in the queue will be published in order on each spin.
        std::queue<uint8_t> route_event_queue;

        // private helper function to add a new route event into event queue
        void publishRouteEvent(uint8_t event_type);        

        // maximum cross track error which can trigger left route event
        double cross_track_dist_;

        // counter to record how many times vehicle's position exceeds crosstrack distance
        int cte_count_ = 0;

        int cte_count_max_;

        // destination points in map
        std::vector<lanelet::BasicPoint2d> destination_points_in_map_;

        // The current map projection for lat/lon to map frame conversion
        boost::optional<std::string> map_proj_;

        geometry_msgs::msg::TransformStamped tf_;
        tf2::Stamped<tf2::Transform> frontbumper_transform_;

        // TF listenser
        tf2_ros::Buffer& tf2_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

        // Logger interface
        rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_;

        rclcpp::Clock::SharedPtr clock_;
    };

} // namespace route