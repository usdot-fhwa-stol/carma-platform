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

#include "route/route_generator_worker.hpp"

namespace route {

    RouteGeneratorWorker::RouteGeneratorWorker(tf2_ros::Buffer& tf2_buffer)
        : tf2_buffer_(tf2_buffer) {}
    
    void RouteGeneratorWorker::setWorldModelPtr(carma_wm::WorldModelConstPtr wm)
    {
        this->world_model_ = wm;
    }

    void RouteGeneratorWorker::setLoggerInterface(rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger)
    {
        this->logger_ = logger;
        this->rs_worker_.setLoggerInterface(logger);
    }

    void RouteGeneratorWorker::setClock(rclcpp::Clock::SharedPtr clock)
    {
        this->clock_ = clock;
    }

    lanelet::Optional<lanelet::routing::Route> RouteGeneratorWorker::routing(const lanelet::BasicPoint2d start,
                                                                             const std::vector<lanelet::BasicPoint2d>& via,
                                                                             const lanelet::BasicPoint2d end,
                                                                             const lanelet::LaneletMapConstPtr map_pointer,
                                                                             const carma_wm::LaneletRoutingGraphConstPtr graph_pointer) const
    {
        // find start lanelet
        auto start_lanelet_vector = lanelet::geometry::findNearest(map_pointer->laneletLayer, start, 1);
        // check if there are any lanelets in the map
        if(start_lanelet_vector.empty())
        {
            RCLCPP_ERROR_STREAM(logger_->get_logger(), "Found no lanelets in the map. Routing cannot be done.");
            return lanelet::Optional<lanelet::routing::Route>();
        }
        // extract starting lanelet
        auto start_lanelet = lanelet::ConstLanelet(start_lanelet_vector[0].second.constData());
        // find end lanelet
        auto end_lanelet_vector = lanelet::geometry::findNearest(map_pointer->laneletLayer, end, 1);
        // extract end lanelet
        auto end_lanelet = lanelet::ConstLanelet(end_lanelet_vector[0].second.constData());
        // find all via lanelets
        lanelet::ConstLanelets via_lanelets_vector;
        for(lanelet::BasicPoint2d point : via)
        {
            auto via_lanelet_vector = lanelet::geometry::findNearest(map_pointer->laneletLayer, point, 1);
            via_lanelets_vector.emplace_back(lanelet::ConstLanelet(via_lanelet_vector[0].second.constData()));
        }
        // routing
        return graph_pointer->getRouteVia(start_lanelet, via_lanelets_vector, end_lanelet);
    }

    void RouteGeneratorWorker::setReroutingChecker(const std::function<bool()> inputFunction)
    {
        reroutingChecker=inputFunction;
    }

    bool RouteGeneratorWorker::getAvailableRouteCb(const std::shared_ptr<rmw_request_id_t>,
                                const std::shared_ptr<carma_planning_msgs::srv::GetAvailableRoutes::Request>,
                                std::shared_ptr<carma_planning_msgs::srv::GetAvailableRoutes::Response> resp)
    {   
        // Return if the the directory specified by route_file_path_ does not exist
        if(!boost::filesystem::exists(boost::filesystem::path(this->route_file_path_)))
        {
            RCLCPP_ERROR_STREAM(logger_->get_logger(), "No directory exists at " << route_file_path_);
            return true;
        }
 
        // Read all route files in the given directory
        boost::filesystem::directory_iterator end_point;
        for(boost::filesystem::directory_iterator itr(boost::filesystem::path(this->route_file_path_)); itr != end_point; ++itr)
        {
            // Skip if the iterator has landed on a folder
            if(boost::filesystem::is_directory(itr->status()))
            {
                continue;
            }

            auto full_file_name = itr->path().filename().generic_string();
            carma_planning_msgs::msg::Route route_msg;

            // Skip if '.csv' is not found in the file name
            if(full_file_name.find(".csv") == std::string::npos)
            { 
                continue;
            }          

            // Assume route files ending with ".csv", before that is the actual route name
            route_msg.route_id = full_file_name.substr(0, full_file_name.find(".csv"));
            std::ifstream fin(itr->path().generic_string());
            std::string dest_name;
            if(fin.is_open())
            {
                while (!fin.eof())
                {
                    std::string temp;
                    std::getline(fin, temp);
                    if(temp != "") dest_name = temp;
                }
                fin.close();
            } 
            else
            {
                RCLCPP_ERROR_STREAM(logger_->get_logger(), "File open failed...");
            }
            auto last_comma = dest_name.find_last_of(',');
            if(!std::isdigit(dest_name.substr(last_comma + 1).at(0)))
            {
                route_msg.route_name = dest_name.substr(last_comma + 1);
                resp->available_routes.push_back(move(route_msg));
            }
        }
            
        //after route path object is available to select, worker will able to transit state and provide route selection service
        if(this->rs_worker_.getRouteState() == RouteStateWorker::RouteState::LOADING) 
        {
            this->rs_worker_.onRouteEvent(RouteStateWorker::RouteEvent::ROUTE_LOADED);
            publishRouteEvent(carma_planning_msgs::msg::RouteEvent::ROUTE_LOADED);
        }

        return true;
    }

    void RouteGeneratorWorker::setRouteFilePath(const std::string& path)
    {
        this->route_file_path_ = path;
        // after route path is set, worker will able to transit state and provide route selection service
        this->rs_worker_.onRouteEvent(RouteStateWorker::RouteEvent::ROUTE_LOADED);
        publishRouteEvent(carma_planning_msgs::msg::RouteEvent::ROUTE_LOADED);
    }

    bool RouteGeneratorWorker::setActiveRouteCb(const std::shared_ptr<rmw_request_id_t>,
                                const std::shared_ptr<carma_planning_msgs::srv::SetActiveRoute::Request> req,
                                std::shared_ptr<carma_planning_msgs::srv::SetActiveRoute::Response> resp)
    {   
        // only allow a new route to be activated in route selection state
        if(this->rs_worker_.getRouteState() != RouteStateWorker::RouteState::SELECTION)
        {
            RCLCPP_ERROR_STREAM(logger_->get_logger(), "System is already following a route.");
            resp->error_status = carma_planning_msgs::srv::SetActiveRoute::Response::ALREADY_FOLLOWING_ROUTE;

            return true;
        }

        RCLCPP_DEBUG_STREAM(logger_->get_logger(), "Valid state proceeding with selection");
        	
        // entering to routing state once destinations are picked
        this->rs_worker_.onRouteEvent(RouteStateWorker::RouteEvent::ROUTE_SELECTED);
        publishRouteEvent(carma_planning_msgs::msg::RouteEvent::ROUTE_SELECTED);

        if (!vehicle_pose_) {
            RCLCPP_ERROR_STREAM(logger_->get_logger(), "No vehicle position. Routing cannot be completed.");
            resp->error_status = carma_planning_msgs::srv::SetActiveRoute::Response::ROUTING_FAILURE;
            this->rs_worker_.onRouteEvent(RouteStateWorker::RouteEvent::ROUTE_GEN_FAILED);
            publishRouteEvent(carma_planning_msgs::msg::RouteEvent::ROUTE_GEN_FAILED);
            return true;
        }

        // Check if the map projection is available
        if (!map_proj_) {
            RCLCPP_ERROR_STREAM(logger_->get_logger(), "Could not generate route as there was no map projection available");
            resp->error_status = carma_planning_msgs::srv::SetActiveRoute::Response::ROUTING_FAILURE;
            this->rs_worker_.onRouteEvent(RouteStateWorker::RouteEvent::ROUTE_GEN_FAILED);
            publishRouteEvent(carma_planning_msgs::msg::RouteEvent::ROUTE_GEN_FAILED);
            return true;
        }

        // load destination points in map frame
        std::vector<lanelet::BasicPoint3d> destination_points;
        if(req->choice == carma_planning_msgs::srv::SetActiveRoute::Request::ROUTE_ID)
        {   
            RCLCPP_INFO_STREAM(logger_->get_logger(), "set_active_route_cb: Selected Route ID: " << req->route_id);
            std::vector<carma_v2x_msgs::msg::Position3D> gps_destination_points = loadRouteDestinationGpsPointsFromRouteId(req->route_id);
            destination_points = loadRouteDestinationsInMapFrame(gps_destination_points);
        }
        else if(req->choice == carma_planning_msgs::srv::SetActiveRoute::Request::DESTINATION_POINTS_ARRAY)
        {
            RCLCPP_INFO_STREAM(logger_->get_logger(), "set_active_route_cb: Destination Points array of size " << req->destination_points.size());
            destination_points = loadRouteDestinationsInMapFrame(req->destination_points);
        }

        // Check that the requested route is valid with at least one destination point
        if(destination_points.size() < 1)
        {
            RCLCPP_ERROR_STREAM(logger_->get_logger(), "Provided route contains no destination points. Routing cannot be completed.");
            resp->error_status = carma_planning_msgs::srv::SetActiveRoute::Response::ROUTE_FILE_ERROR;
            this->rs_worker_.onRouteEvent(RouteStateWorker::RouteEvent::ROUTE_GEN_FAILED);
            publishRouteEvent(carma_planning_msgs::msg::RouteEvent::ROUTE_GEN_FAILED);
            return true;
        }

        if (!world_model_ || !world_model_->getMap()) {
            RCLCPP_ERROR_STREAM(logger_->get_logger(), "World model has not been initialized.");
            resp->error_status = carma_planning_msgs::srv::SetActiveRoute::Response::ROUTING_FAILURE;
            this->rs_worker_.onRouteEvent(RouteStateWorker::RouteEvent::ROUTE_GEN_FAILED);
            publishRouteEvent(carma_planning_msgs::msg::RouteEvent::ROUTE_GEN_FAILED);
            return true;
        }

        // convert points in 2d to map frame
        destination_points_in_map_ = lanelet::utils::transform(destination_points, [](auto a) { return lanelet::traits::to2D(a); });

        // Add vehicle as first destination point
        auto destination_points_in_map_with_vehicle = destination_points_in_map_;
            
        lanelet::BasicPoint2d vehicle_position(vehicle_pose_->pose.position.x, vehicle_pose_->pose.position.y);
        destination_points_in_map_with_vehicle.insert(destination_points_in_map_with_vehicle.begin(), vehicle_position);

        int idx = 0;
        // validate if the points are geometrically in the map
        for (auto pt : destination_points_in_map_with_vehicle)
        {
            if ((world_model_->getLaneletsFromPoint(pt, 1)).empty())
            {
                RCLCPP_ERROR_STREAM(logger_->get_logger(), "Route Generator: " << idx 
                        << "th destination point is not in the map, x: " << pt.x() << " y: " << pt.y());
                resp->error_status = carma_planning_msgs::srv::SetActiveRoute::Response::ROUTE_FILE_ERROR;
                this->rs_worker_.onRouteEvent(RouteStateWorker::RouteEvent::ROUTE_GEN_FAILED);
                publishRouteEvent(carma_planning_msgs::msg::RouteEvent::ROUTE_GEN_FAILED);
                return true;
            }
            idx ++;
        }
            
        // get route graph from world model object
        auto p = world_model_->getMapRoutingGraph();
        // generate a route
        auto route = routing(destination_points_in_map_with_vehicle.front(),
                            std::vector<lanelet::BasicPoint2d>(destination_points_in_map_with_vehicle.begin() + 1, destination_points_in_map_with_vehicle.end() - 1),
                            destination_points_in_map_with_vehicle.back(),
                            world_model_->getMap(), world_model_->getMapRoutingGraph());
        // check if route succeeded
        if(!route)
        {
            RCLCPP_ERROR_STREAM(logger_->get_logger(), "Cannot find a route passing all destinations.");
            resp->error_status = carma_planning_msgs::srv::SetActiveRoute::Response::ROUTING_FAILURE;
            this->rs_worker_.onRouteEvent(RouteStateWorker::RouteEvent::ROUTE_GEN_FAILED);
            publishRouteEvent(carma_planning_msgs::msg::RouteEvent::ROUTE_GEN_FAILED);
            return true;
        }

        if (checkForDuplicateLaneletsInShortestPath(route.get()))
        {
            RCLCPP_ERROR_STREAM(logger_->get_logger(), "At least one duplicate Lanelet ID occurs in the shortest path. Routing cannot be completed.");
            resp->error_status = carma_planning_msgs::srv::SetActiveRoute::Response::ROUTING_FAILURE;
            this->rs_worker_.onRouteEvent(RouteStateWorker::RouteEvent::ROUTE_GEN_FAILED);
            publishRouteEvent(carma_planning_msgs::msg::RouteEvent::ROUTE_GEN_FAILED);
            return true;
        }

        // Specify the end point of the route that is inside the last lanelet
        lanelet::Point3d end_point{lanelet::utils::getId(), destination_points_in_map_with_vehicle.back().x(), destination_points_in_map_with_vehicle.back().y(), 0};

        route->setEndPoint(end_point);

        // update route message
        route_msg_ = composeRouteMsg(route);

        for(auto id : route_msg_.route_path_lanelet_ids)
        {
            auto ll = world_model_->getMap()->laneletLayer.get(id);
            route_llts.push_back(ll);
        }

        route_msg_.route_name = req->route_id;
        route_marker_msg_ = composeRouteMarkerMsg(route);
        route_msg_.header.stamp = clock_->now();
        route_msg_.header.frame_id = "map";
        route_msg_.map_version = world_model_->getMapVersion();
        // since routing is done correctly, transit to route following state
        this->rs_worker_.onRouteEvent(RouteStateWorker::RouteEvent::ROUTE_STARTED);
        publishRouteEvent(carma_planning_msgs::msg::RouteEvent::ROUTE_STARTED);
        // set publish flag such that updated msg will be published in the next spin
        new_route_msg_generated_ = true;
        return true;
    }

    bool RouteGeneratorWorker::checkForDuplicateLaneletsInShortestPath(const lanelet::routing::Route& route) const
    {
        // Create a vector for the lanelet IDs in the shortest path
        std::vector<lanelet::Id> shortest_path_lanelet_ids;

        // Iterate through the shortest path to populate shortest_path_lanelet_ids with lanelet IDs
        for(const auto& ll : route.shortestPath())
        {
            shortest_path_lanelet_ids.push_back(ll.id());
        }

        // Verify that there are no duplicate lanelet IDs in the shortest path
        std::sort(shortest_path_lanelet_ids.begin(), shortest_path_lanelet_ids.end());
        
        if (std::adjacent_find(shortest_path_lanelet_ids.begin(), shortest_path_lanelet_ids.end()) != shortest_path_lanelet_ids.end())
        {
            // Route's shortest path contains duplicate lanelet IDs
            return true;
        }

        // Route's shortest path does not duplicate lanelet IDs
        return false;
    }

    std::vector<lanelet::BasicPoint3d> RouteGeneratorWorker::loadRouteDestinationsInMapFrame(const std::vector<carma_v2x_msgs::msg::Position3D>& destinations) const
    {
        if (!map_proj_) {
            throw std::invalid_argument("loadRouteDestinationsInMapFrame (using destination points array) before map projection was set");
        }
        
        lanelet::projection::LocalFrameProjector projector(map_proj_.get().c_str()); // Build map projector

        // Process each point in 'destinations'
        std::vector<lanelet::BasicPoint3d> destination_points;
        for (const auto& destination : destinations)
        {
            lanelet::GPSPoint coordinate;
            coordinate.lon = destination.longitude;
            coordinate.lat = destination.latitude;
            
            if(destination.elevation_exists)
            {
                coordinate.ele = destination.elevation;
            }
            else
            {
                coordinate.ele = 0.0;
            }

            destination_points.emplace_back(projector.forward(coordinate));
        }

        return destination_points;
    }

    std::vector<carma_v2x_msgs::msg::Position3D> RouteGeneratorWorker::loadRouteDestinationGpsPointsFromRouteId(const std::string& route_id) const
    {
        // compose full path of the route file
        std::string route_file_name = route_file_path_ + route_id + ".csv";
        std::ifstream fs(route_file_name);
        std::string line;
        
        // read each line in route file (if any)
        std::vector<carma_v2x_msgs::msg::Position3D> destination_points;
        while(std::getline(fs, line))
        {
            carma_v2x_msgs::msg::Position3D gps_point;

            // lat lon and elev is seperated by comma
            auto comma = line.find(",");
            // convert lon value in degrees from string
            gps_point.longitude = std::stof(line.substr(0, comma));
            line.erase(0, comma + 1);
            comma = line.find(",");
            // convert lat value in degrees from string
            gps_point.latitude = std::stof(line.substr(0, comma));
            // elevation is in meters
            line.erase(0, comma + 1);
            comma = line.find(",");
            gps_point.elevation = std::stof(line.substr(0, comma));
            gps_point.elevation_exists = true;

            destination_points.push_back(gps_point);
        }

        return destination_points;
    }

    visualization_msgs::msg::Marker RouteGeneratorWorker::composeRouteMarkerMsg(const lanelet::Optional<lanelet::routing::Route>& route)
    {
        std::vector<lanelet::ConstPoint3d> points;
        auto end_point_3d = route.get().getEndPoint();
        auto last_ll = route.get().shortestPath().back();
        double end_point_downtrack = carma_wm::geometry::trackPos(last_ll, {end_point_3d.x(), end_point_3d.y()}).downtrack;
        double lanelet_downtrack = carma_wm::geometry::trackPos(last_ll, last_ll.centerline().back().basicPoint2d()).downtrack;
        // get number of points to display using ratio of the downtracks
        auto points_until_end_point = int (last_ll.centerline().size() * (end_point_downtrack / lanelet_downtrack));
  
        for(const auto& ll : route.get().shortestPath())
        {
            if (ll.id() == last_ll.id())
            {
                for (int i = 0; i < points_until_end_point; i++)
                {
                    points.push_back(ll.centerline()[i]);
                }
                continue;
            }
            for(const auto& pt : ll.centerline())
            {
                points.push_back(pt);
            }
        }

        route_marker_msg_.points={};

        // create the marker msgs
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Time();
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;//
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.ns = "route_visualizer";

        marker.scale.x = 0.65;
        marker.scale.y = 0.65;
        marker.scale.z = 0.65;
        marker.frame_locked = true;

        marker.id = 0;
        marker.color.r = 1.0F;
        marker.color.g = 1.0F;
        marker.color.b = 1.0F;
        marker.color.a = 1.0F;

        if (points.empty())
        {
            RCLCPP_WARN_STREAM(logger_->get_logger(), "No central line points! Returning");
            return route_marker_msg_;
        }
 
        for (int i = 0; i < points.size(); i=i+5)
        {
            geometry_msgs::msg::Point temp_point;
            temp_point.x = points[i].x();
            temp_point.y = points[i].y();
            temp_point.z = 1; //to show up on top of the lanelet lines
            
            marker.points.push_back(temp_point);
        }
        new_route_marker_generated_ = true;
        return marker;
    }

    carma_planning_msgs::msg::Route RouteGeneratorWorker::composeRouteMsg(const lanelet::Optional<lanelet::routing::Route>& route) const
    {
        carma_planning_msgs::msg::Route msg;
        // iterate through the shortest path to populate shortest_path_lanelet_ids
        for(const auto& ll : route.get().shortestPath())
        {
            msg.shortest_path_lanelet_ids.push_back(ll.id());
        }
        // iterate through all lanelet in the route to populate route_path_lanelet_ids
        for(const auto& ll : route.get().laneletSubmap()->laneletLayer)
        {
            msg.route_path_lanelet_ids.push_back(ll.id());
        }
        msg.end_point.x  = route->getEndPoint().x();
        msg.end_point.y  = route->getEndPoint().y();
        msg.end_point.z  = route->getEndPoint().z();

        return msg;
    }

    bool RouteGeneratorWorker::abortActiveRouteCb(const std::shared_ptr<rmw_request_id_t>,
                                const std::shared_ptr<carma_planning_msgs::srv::AbortActiveRoute::Request>,
                                std::shared_ptr<carma_planning_msgs::srv::AbortActiveRoute::Response> resp)
    {
        // only make sense to abort when it is in route following state
        if(this->rs_worker_.getRouteState() == RouteStateWorker::RouteState::FOLLOWING)
        {
            this->rs_worker_.onRouteEvent(RouteStateWorker::RouteEvent::ROUTE_ABORTED);
            resp->error_status = carma_planning_msgs::srv::AbortActiveRoute::Response::NO_ERROR;
            publishRouteEvent(carma_planning_msgs::msg::RouteEvent::ROUTE_ABORTED);
            route_msg_ = carma_planning_msgs::msg::Route{};
        } else {
            // service call successed but there is not active route
            resp->error_status = carma_planning_msgs::srv::AbortActiveRoute::Response::NO_ACTIVE_ROUTE;
        }
        return true;
    }

    void RouteGeneratorWorker::initializeBumperTransformLookup() 
    {
        tf2_listener_.reset(new tf2_ros::TransformListener(tf2_buffer_));
        tf2_buffer_.setUsingDedicatedThread(true);
    }

    void RouteGeneratorWorker::bumperPoseCb()
    {
        try
        {
            tf_ = tf2_buffer_.lookupTransform("map", "vehicle_front", rclcpp::Time(0,0), rclcpp::Duration(1.0*1e9)); //save to local copy of transform 0.1 sec timeout
            tf2::fromMsg(tf_, frontbumper_transform_);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_STREAM(logger_->get_logger(), ex.what());
        }
        
        geometry_msgs::msg::PoseStamped updated_vehicle_pose;
        updated_vehicle_pose.pose.position.x = frontbumper_transform_.getOrigin().getX();
        updated_vehicle_pose.pose.position.y = frontbumper_transform_.getOrigin().getY();
        updated_vehicle_pose.pose.position.z = frontbumper_transform_.getOrigin().getZ();
        vehicle_pose_ = updated_vehicle_pose; 
            
        if(this->rs_worker_.getRouteState() == RouteStateWorker::RouteState::FOLLOWING) {
            // convert from pose stamp into lanelet basic 2D point
            current_loc_ = lanelet::BasicPoint2d(vehicle_pose_->pose.position.x, vehicle_pose_->pose.position.y);
            // get dt ct from world model
            carma_wm::TrackPos track(0.0, 0.0);
            try {
                track = this->world_model_->routeTrackPos(current_loc_);
            } catch (std::invalid_argument ex) {
                RCLCPP_WARN_STREAM(logger_->get_logger(), "Routing has finished but carma_wm has not receive it!");
                return;
            }

            // Return if world model has not yet been updated with the current active route
            if ((this->world_model_->getRouteName()).compare(route_msg_.route_name) != 0) {
                RCLCPP_WARN_STREAM(logger_->get_logger(), "Current active route name is " << route_msg_.route_name << ", WorldModel is using " << this->world_model_->getRouteName());
                return;
            }

            auto current_lanelet = getClosestLaneletFromRouteLanelets(current_loc_);
            auto lanelet_track = carma_wm::geometry::trackPos(current_lanelet, current_loc_);
            ll_id_ = current_lanelet.id();
            ll_crosstrack_distance_ = lanelet_track.crosstrack;
            ll_downtrack_distance_ = lanelet_track.downtrack;
            current_crosstrack_distance_ = track.crosstrack;
            current_downtrack_distance_ = track.downtrack;
            // Determine speed limit
            lanelet::Optional<carma_wm::TrafficRulesConstPtr> traffic_rules = world_model_->getTrafficRules();
            
            if (traffic_rules) 
            {
                auto laneletIterator = world_model_->getMap()->laneletLayer.find(ll_id_);
                if (laneletIterator != world_model_->getMap()->laneletLayer.end()) {
                    speed_limit_ = (*traffic_rules)->speedLimit(*laneletIterator).speedLimit.value();
                } 
                else 
                {
                    RCLCPP_ERROR_STREAM(logger_->get_logger(), "Failed to set the current speed limit. The lanelet_id: "
                        << ll_id_ << " could not be matched with a lanelet in the map. The previous speed limit of "
                        << speed_limit_ << " will be used.");
                }
                
            } 
            else 
            {
                RCLCPP_ERROR_STREAM(logger_->get_logger(), "Failed to set the current speed limit. Valid traffic rules object could not be built.");
            }
            std::shared_ptr<geometry_msgs::msg::PoseStamped> pose_ptr(new geometry_msgs::msg::PoseStamped(*vehicle_pose_));
            
            // check if we left the seleted route by cross track error
            if (crosstrackErrorCheck(pose_ptr, current_lanelet))
            {
                this->rs_worker_.onRouteEvent(RouteStateWorker::RouteEvent::ROUTE_DEPARTED);
                publishRouteEvent(carma_planning_msgs::msg::RouteEvent::ROUTE_DEPARTED);
            }

            // check if we reached our destination be remaining down track distance            
            double route_length_2d = world_model_->getRouteEndTrackPos().downtrack;
            if((current_downtrack_distance_ > route_length_2d - down_track_target_range_ && current_speed_ < epsilon_) || (current_downtrack_distance_ > route_length_2d))
            {
                this->rs_worker_.onRouteEvent(RouteStateWorker::RouteEvent::ROUTE_COMPLETED);
                publishRouteEvent(carma_planning_msgs::msg::RouteEvent::ROUTE_COMPLETED);
            }
        }
    }

    void RouteGeneratorWorker::twistCb(geometry_msgs::msg::TwistStamped::UniquePtr msg)
    {
        current_speed_ = msg->twist.linear.x;
    }

    void RouteGeneratorWorker::georeferenceCb(std_msgs::msg::String::UniquePtr msg)
    {
        map_proj_ = msg->data;
    }

    void RouteGeneratorWorker::setPublishers(const carma_ros2_utils::PubPtr<carma_planning_msgs::msg::RouteEvent>& route_event_pub,
                        const carma_ros2_utils::PubPtr<carma_planning_msgs::msg::RouteState>& route_state_pub,
                        const carma_ros2_utils::PubPtr<carma_planning_msgs::msg::Route>& route_pub,
                        const carma_ros2_utils::PubPtr<visualization_msgs::msg::Marker>& route_marker_pub)
    {
        route_event_pub_ = route_event_pub;
        route_state_pub_ = route_state_pub;
        route_pub_ = route_pub;
        route_marker_pub_= route_marker_pub;
    }

    void RouteGeneratorWorker::setDowntrackDestinationRange(double dt_dest_range)
    {
        this->down_track_target_range_ = dt_dest_range;
    }

    void RouteGeneratorWorker::publishRouteEvent(uint8_t event_type)
    {
        route_event_queue.push(event_type);
    }
    
    lanelet::Optional<lanelet::routing::Route> RouteGeneratorWorker::rerouteAfterRouteInvalidation(const std::vector<lanelet::BasicPoint2d>& destination_points_in_map)
    {
        std::vector<lanelet::BasicPoint2d> destination_points_in_map_temp;
        
        for(const auto &i:destination_points_in_map) // Identify all route points that we have not yet passed
        {
            double destination_down_track=world_model_->routeTrackPos(i).downtrack;
            
            if( current_downtrack_distance_< destination_down_track)
            {
                destination_points_in_map_temp.push_back(i);
                RCLCPP_DEBUG_STREAM(logger_->get_logger(), "current_downtrack_distance_:" << current_downtrack_distance_);
                RCLCPP_DEBUG_STREAM(logger_->get_logger(), "destination_down_track:" << destination_down_track);
            }
        }  
        
        destination_points_in_map_ = destination_points_in_map_temp; // Update our route point list
        
        RCLCPP_DEBUG_STREAM(logger_->get_logger(), "New destination_points_in_map.size:" << destination_points_in_map_.size());

        auto route=routing(current_loc_, // Route from current location through future destinations
                            std::vector<lanelet::BasicPoint2d>(destination_points_in_map_.begin(), destination_points_in_map_.end() - 1),
                            destination_points_in_map_.back(),
                            world_model_->getMap(), world_model_->getMapRoutingGraph());

        return route;
    }

    bool RouteGeneratorWorker::spinCallback()
    {
        // Update vehicle position
        bumperPoseCb();

        if(reroutingChecker()==true)
        {
           RCLCPP_DEBUG_STREAM(logger_->get_logger(), "Rerouting required");
           this->rs_worker_.onRouteEvent(RouteStateWorker::RouteEvent::ROUTE_INVALIDATION);
           publishRouteEvent(carma_planning_msgs::msg::RouteEvent::ROUTE_INVALIDATION);
           auto route = rerouteAfterRouteInvalidation(destination_points_in_map_);

           // check if route successed
           if(!route)
            {
                RCLCPP_ERROR_STREAM(logger_->get_logger(), "Cannot find a route passing all destinations.");
                this->rs_worker_.onRouteEvent(RouteStateWorker::RouteEvent::ROUTE_GEN_FAILED);
                publishRouteEvent(carma_planning_msgs::msg::RouteEvent::ROUTE_GEN_FAILED);
                return true;
            }
            else if(checkForDuplicateLaneletsInShortestPath(route.get()))
            {
                RCLCPP_ERROR_STREAM(logger_->get_logger(), "At least one duplicate Lanelet ID occurs in the shortest path. Routing cannot be completed.");
                this->rs_worker_.onRouteEvent(RouteStateWorker::RouteEvent::ROUTE_GEN_FAILED);
                publishRouteEvent(carma_planning_msgs::msg::RouteEvent::ROUTE_GEN_FAILED);
                return true;
            }
            else
            {
                this->rs_worker_.onRouteEvent(RouteStateWorker::RouteEvent::ROUTE_STARTED);
                publishRouteEvent(carma_planning_msgs::msg::RouteEvent::ROUTE_STARTED);  
            }    
            std::string original_route_name = route_msg_.route_name;
            route_msg_ = composeRouteMsg(route);
            route_msg_.route_name = original_route_name;
            route_msg_.is_rerouted = true;
            route_msg_.map_version = world_model_->getMapVersion();
            route_marker_msg_ = composeRouteMarkerMsg(route);
            new_route_msg_generated_ = true;
            new_route_marker_generated_ = true;
        }
        
        // publish new route and set new route flag back to false
        if(new_route_msg_generated_ && new_route_marker_generated_)
        {
            route_pub_->publish(route_msg_);
            route_marker_pub_->publish(route_marker_msg_);
            new_route_msg_generated_ = false;
            new_route_marker_generated_ = false;
            route_msg_.is_rerouted = false;
        }
        // publish route state messsage if a route is selected
        if(route_msg_.route_name != "")
        {
            carma_planning_msgs::msg::RouteState state_msg;
            state_msg.header.stamp = clock_->now();
            state_msg.route_id = route_msg_.route_name;
            state_msg.cross_track = current_crosstrack_distance_;
            state_msg.down_track = current_downtrack_distance_;
            state_msg.lanelet_downtrack = ll_downtrack_distance_;            
            state_msg.state = this->rs_worker_.getRouteState();
            state_msg.lanelet_id = ll_id_;
            state_msg.speed_limit = speed_limit_;
            route_state_pub_->publish(state_msg);
        }
        // publish route event in order if any
        while(!route_event_queue.empty())
        {
            route_event_msg_.event = route_event_queue.front();
            RCLCPP_INFO_STREAM(logger_->get_logger(), "Publishing a route event!");
            route_event_pub_->publish(route_event_msg_);
            route_event_queue.pop();
        }
        return true; 
    }

    bool RouteGeneratorWorker::crosstrackErrorCheck(const std::shared_ptr<geometry_msgs::msg::PoseStamped>& msg, lanelet::ConstLanelet current)
    {
        lanelet::BasicPoint2d position;

        position.x()= msg->pose.position.x;
        position.y()= msg->pose.position.y;

        if(boost::geometry::within(position, current.polygon2d())) //If vehicle is inside current_lanelet, there is no crosstrack error
        {
            cte_count_ = 0;
            return false;
        }

        RCLCPP_DEBUG_STREAM(logger_->get_logger(), "LLt Polygon Dimensions1: " << current.polygon2d().front().x()<< ", "<< current.polygon2d().front().y());
        RCLCPP_DEBUG_STREAM(logger_->get_logger(), "LLt Polygon Dimensions2: " << current.polygon2d().back().x()<< ", "<< current.polygon2d().back().y());
        RCLCPP_DEBUG_STREAM(logger_->get_logger(), "Distance1: " << boost::geometry::distance(position, current.polygon2d()) << " Max allowed Crosstrack: " << cross_track_dist_ );
    
        if (boost::geometry::distance(position, current.polygon2d()) > cross_track_dist_) //Evaluate lanelet crosstrack distance from vehicle
            {
                cte_count_++;

                if(cte_count_ > cte_count_max_) //If the distance exceeds the crosstrack distance a certain number of times, report that the route has been departed
                    {
                        cte_count_ = 0;
                        return true;
                    }
                else 
                    return false;
            }
        else
            {
                cte_count_ = 0;
                return false;
            }
 
    }



    lanelet::ConstLanelet RouteGeneratorWorker::getClosestLaneletFromRouteLanelets(lanelet::BasicPoint2d position) const
    {
        double min = std::numeric_limits<double>::infinity();
        lanelet::ConstLanelet min_llt;
        for (const auto& i: route_llts)
         {
            double dist = boost::geometry::distance(position, i.polygon2d());
            if (dist < min)
            {
                min = dist;
                min_llt = i;
            }
     }
    return min_llt;
    }

    void RouteGeneratorWorker::setCrosstrackErrorDistance(double cte_dist)
    {
        cross_track_dist_ = cte_dist;
    }

     void RouteGeneratorWorker::setCrosstrackErrorCountMax(int cte_max)
    {
        cte_count_max_ = cte_max;

    }

    void RouteGeneratorWorker::addLanelet(lanelet::ConstLanelet llt)
    {
        route_llts.push_back(llt);
    }

} // route