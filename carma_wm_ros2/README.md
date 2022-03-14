# carma_wm_ros2

This package provides the carma_wm C++ library which manages and simplifies accessing and querying the road network that the vehicle is operating in. This package makes heavy use of the [Lanelet2 C++ Library](https://github.com/fzi-forschungszentrum-informatik/Lanelet2) to represent road geometry and communicate with the rest of the CARMA Platform. A copy of the Lanelet2 License can be found [here](doc/LANELET2_LICENSE.md).  

## Library Role

The carma_wm library design was inspired by the [tf2 library](http://wiki.ros.org/tf2) in that it provides automatic subscription and message callbacks for semantic map and route updates in CARMA. Beyond this the library is designed for support a wider range of route-based computations than Lanelet2 provides by default, however it is expected that users will learn and use the Lanelet2 API directly for most non-route based needs. The library provides read-only access to the map and route objects. Map and route updating is handled by other components in the CARMA Platform.  

## CARMA Lanelet2 OSM File Format Changes

By default Lanelet2 supports a modified version of the OpenStreetMaps (OSM) file format to describe its roadways. See [here](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_maps/README.md) for an introduction. Part of this standard describes how road regulations are defined by interpreting tags on various Lanelet2 primitives. In addition, regulatory elements also fill a similar role. Unfortunately, this creates a confusing situation where some regulations are tied intrinsically to the physical markings on the road. For example, a double yellow line could never be crossed even if nearby signs instructed the user to do so around a work zone. To resolve this, while also preserving the description of the physical markings on the road, CARMA Platform supports an additional set of [regulatory elements](https://github.com/usdot-fhwa-stol/autoware.ai/blob/carma-develop/common/lanelet2_extension/docs/RegulatoryElements.md). Using these regulations separates all road regulations from physical road markings allowing for situations such as the work zone use case described above. In addition, all Lanelet2 Maps must follow the Autoware.ai tagging specification rules described in this document and [here](https://github.com/usdot-fhwa-stol/autoware.ai/blob/carma-develop/common/lanelet2_extension/docs/lanelet2_format_extension.md) due to the use of legacy map handling nodes. This is a requirement which may be depercated in the future. If a Lanelet2 map is loaded which does not contain these elements the carma_wm_ctrl package will make a best effort attempt to add them using Lanelet2's default tagging rules before forwarding the map to components using the carma_wm library.

## Route Definition

LaneLet2 provides its own routing module. Details of this module can be found [here](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/tree/master/lanelet2_routing) and should be reviewed before continuing in with this section. Lanelet2 generates a route by finding the shortest path of LaneLets between two points according to a user defined cost function. Then it will identify all the LaneLets which lie along the same roads as this path and include them in the description. The sequence of LaneLets which describe the original shortest path will also be preserved inside the route object. This can be seen below.

![](doc/media/path_vs_route.png)
Depiction of shortest path compared with Lanelet2 route
Images sourced from Lanelet2 routing documentation found [here](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/tree/master/lanelet2_routing).

To support the concept of downtrack and crosstrack distance often used for planning in cooperative applications the reference line will be defined by the center lines which lie along the route’s shortest path. This will remove ambiguity in where the reference line is located on the road as shown below.
 
![](doc/media/DowntrackReference.png)
Route line for calculating downtrack distance

When there are lane changes that occur along the route the line of reference will move to the following lane after the lane change lanelet as shown below. This will prevent the length of the adjacent lanelet being double counted.  
 
![](doc/media/ReferenceLine.png)
Route reference line at lane change location. Image sourced from Lanelet2 publication found [here](https://www.mrt.kit.edu/z/publ/download/2018/Poggenhans2018Lanelet2.pdf)

## Example Usage

### Initialization

Users should initialize the carma_wm by first creating an instance of the [WMListener](include/carma_wm/WMListener.h) object. This will automatically subscribe to the ```semantic_map``` and ```route``` topics which will provide map and route updates. By default the WMListener is single threaded and will only trigger callbacks when ```ros::spin()``` is called. However, as map and route updates can be time consuming there is a multi-threaded mode which can be enabled using WMListener constructor. This will use a ```ros::AsyncSpinner``` to update the map and route in the background. When this happens the user should take care to ensure thread safety when performing map or route access through the use of the ```WMListener.getLock()``` method.  

Once the user decides they need to access map or route information, they will do so through an instance of the [WorldModel](include/carma_wm/WorldModel.h) interface. This provides read access to map and route objects as well as functions for quickly computing downtrack or crosstrack distances. An instance of the WorldModel can be acquired using the ```WMListener.getWorldModel()``` method.  The WorldModel object is not thread safe on its own which is why usage of the ```WMListener.getLock()``` method is critical when using multi-threaded mode.

#### Single Threaded Example Code

```c++
#include <rclcpp/rclcpp.hpp>
#include <carma_wm_ros2/WMListener.hpp>
#include <carma_wm_ros2/WorldModel.hpp>
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv, "map_user");

    // Create WMListener after initializing ros
    // It is recommended only one instance be created per node

    auto node = std::make_shared<rclcpp::Node>("base_node");

    //Extract node interfaces from base node
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base = node->get_node_base_interface(); 
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging = node->get_node_logging_interface();)
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics = node->get_node_topics_interface();
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_params_ = node->get_node_parameters_interface();
    
    auto wml = std::make_shared<carma_wm::WMListener>(rclcpp::NodeOptions(), node_base, node_logging, node_topics, node_params_); // Create single threaded listener instance. Equivalent to carma_wm::WMListener wm (rclcpp::NodeOptions(), node_base, node_logging, node_topics,false);

    carma_wm::WorldModelConstPtr wm = wml.getWorldModel(); // Get pointer to WorldModel


    if (wml.getRoute()) { // Route object provided a shared_ptr so you can use it to check for availability
        lanelet::Point3d pt;

        carma_wm::TrackPos tc = wm->routeTrackPos(pt); // Get the downtrack and crosstrack position of the provided point on the route
    }
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

```

#### Multi-Threaded Example Code

```c++
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <carma_wm_ros2/WMListener.hpp>
#include <carma_wm_ros2/WorldModel.hpp>
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv, "map_user");

    auto node = std::make_shared<rclcpp::Node>("base_node");

    //Extract node interfaces from base node
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base = node->get_node_base_interface(); 
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging = node->get_node_logging_interface();)
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics = node->get_node_topics_interface();
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_params_ = node->get_node_parameters_interface();

    // Create WMListener after initializing ros
    // It is recommended only one instance be created per node
    auto wml = std::make_shared<carma_wm::WMListener>(rclcpp::NodeOptions(), node_base, node_logging, node_topics, node_params_, true); // Create multi-threaded listener instance by passing true constructor parameter

    carma_wm::WorldModelConstPtr wm = wml.getWorldModel(); // Get pointer to WorldModel

    std::atomic<bool> routeReady(false);
    wml.setRouteCallback([&]() { // User can set callback to trigger when a new route or map is received. Works in single threaded case as well
        routeReady.store(true);
    });


    auto lock = wml.getLock(); // Acquire lock. Now user can safely access map and route data
    if (routeReady) { 
        lanelet::Point3d pt;
        carma_wm::TrackPos tc = wm->routeTrackPos(pt); // Get the downtrack and crosstrack position of the provided point on the route
    }
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
  
    return 0;
}

```

#### Unit Test Example Pseudo Code

To better support unit testing, the user should define their classes or functions to take in the pointer to the world model provided by WMListener.
In the unit test, they can then intialize an instance of CARMAWorldModel, and set the map and route manually. NOTE: CARMAWorldModel should not be used in runtime code as the map route synchronization will need to be manually maintained. 

```c++
TEST(UserTest, someTest)
{
  CARMAWorldModel cmw; // Instantiate writeable world model

  // Build map
  std::vector<lanelet::Point3d> left = {
    getPoint(0, 0, 0), // user created helper function to make lanelet points
    getPoint(0, 1, 0),
  };
  std::vector<lanelet::Point3d> right = {
    getPoint(1, 0, 0),
    getPoint(1, 1, 0),
  };

  auto ll = getLanelet(left, right); // user created helper function to make lanelets
  auto map = lanelet::utils::createMap({ ll }, {});

  cmw.setMap(std::move(map));

  // User can now pass a shared pointer of cmw to whatever they wish to test

}

```
In the unit test, they can then intialize an instance of CARMAWorldModel, and set the map and route manually. NOTE: CARMAWorldModel should not be used in runtime code as the map route synchronization will need to be manually maintained. 

#### Unit Test Example for WMTestLibForGuidance

To better support unit testing for guidance components, a test library is made available with a world model that is with prebaked obstacles and several helper functions. In general, it includes the following :
* Helper functions to create the world from scratch or extend the world in getGuidanceTestMap()
* getGuidanceTestMap gives a simple one way, 3 lane map (25mph speed limit) with one static prebaked obstacle and 
 ```c++
using namespace carma_wm::test
TEST(UserTest, someTest)
{
  // get a premade world model for guidance unit test with default config
  auto cmw_default = getGuidanceTestMap();

  // This premade map is configured through MapOptions class
  // lanelet width and length is configurable. there are 4 lanelets in a single lane and 3 lanes in total. The default measures of a lanelet are 3.7 meter, and 25 meters respectively
  MapOptions mp(5.0, 36);
  auto cmw_configured = getGuidanceTestMap(mp);

  // there is option to either not include prebaked obstacle or set no speed limit
  // MapOptions mp(5.0,36, MapOptions::Obstacle::NONE, MapOptions::SpeedLimit::NONE) also works
  mp.speed_limit_ = MapOptions::SpeedLimit::NONE;
  mp.obstacle_ = MapOptions::Obstacle::NONE;
  auto cmw_configured1 = getGuidanceTestMap(mp);
}

```
 * addObstacle at a specified Cartesian relative to the local origin or Trackpos point relative to specified lanelet Id:
```c++
using namespace carma_wm::test
TEST(UserTest, someTest)
{
  // set lanelet width and length to be 1,1
  MapOptions mp(1,1);
  auto cmw = getGuidanceTestMap(mp);
  // add static obstacle by cartesian coords
  // at 0.5,0.5, which is x,y object center
  addObstacle(0.5,0.5,cmw);
  // or add dynamic obstacle by cartesian coords with
  // {0.25,1.5} predicted coord in the next 100ms (by default)
  // and {0.5,2.5} in the next 100ms etc. 0.75, 0.75 width and length of the object
  addObstacle(0.5,0.5, cmw, {{0.25,1.5}, {0.5,2.5}}, 100, 0.75, 0.75);
  // similarily, add static obstacle by trackpos relative to given lanelet id
  carma_wm::TrackPos tp = {0.5, 0};
  carma_wm::TrackPos tp_pred = {1.5, -0.25};
  addObstacle(tp, 1200, cmw, {}, 100, 0.75, 0.75);
  // or add dynamic one, that is predicted to be at 1.5 downtrack and -0.25 crosstrack from 
  // lanelet is 1200 at the next 100ms. NOTE: This function only works on getGuidanceTestMap
  addObstacle(tp, 1200, cmw, {tp_pred});
}
```
 * set route by giving series of lanelet or their Ids in the map
 * or also overwrite the speed of the entire map
```c++
using namespace carma_wm::test

TEST(UserTest, someTest)
{
  auto cmw = getGuidanceTestMap(mp);
  // set the route explicitly with all lanelet ids
  setRouteByIds({1200,1210,1220,1221,1222,1223}, cmw);
  // or let it route automatically by giving only start and end
  setRouteByIds({1200, 1223}, cmw);
  // we can also give lanelets themselves
  auto ll_1200 = cmw->getMap()->laneletLayer.get(1200);
  auto ll_1223 = cmw->getMap()->laneletLayer.get(1223);
  setRouteByLanelets({ll_100, ll_1223}, cmw);
  // overwrite all the speed limit of the entire road
  setSpeedLimit(25_mph, cmw);
}
```
 * add traffic light into the map
```c++
using namespace carma_wm::test
TEST(UserTest, someTest)
{
  auto cmw = getGuidanceTestMap(mp);
  // Light with default signal timers will be located on lanelet 1200 (entry) and exit at 1203.
  // Notice that multiple entry and exit lanelets can be given.
  lanelet::Id traffic_light_id = lanelet::utils::getId();
  carma_wm::test::addTrafficLight(cmw, traffic_light_id, {1200}, { 1203 });
  
  // also timing can be changed:
  std::vector<std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>> timing_plan =
  {
    std::make_pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>(lanelet::time::timeFromSec(0), lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED), // Just ended green
    std::make_pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>(lanelet::time::timeFromSec(2.0), lanelet::CarmaTrafficSignalState::PROTECTED_CLEARANCE), // 2 sec yellow
    std::make_pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>(lanelet::time::timeFromSec(17.0), lanelet::CarmaTrafficSignalState::STOP_AND_REMAIN), // 15 sec red
    std::make_pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>(lanelet::time::timeFromSec(32.0), lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED) // 15 sec green
  }
  lanelet::Id traffic_light_id = lanelet::utils::getId();
  carma_wm::test::addTrafficLight(cmw, traffic_light_id, {1200}, { 1203 }, timing_plan);
}
```




