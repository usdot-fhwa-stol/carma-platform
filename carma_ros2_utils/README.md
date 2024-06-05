# carma_ros2_utils

This package contains utility libraries for using ROS2 with carma.

## CarmaLifecycleNode

The CarmaLifecycleNode provides default exception handling and SystemAlert handling for components in carma-platform. All new nodes in carma-platform are expected to be built off CarmaLifecycleNode and implemented as components.

### Example

The following example shows how to create a basic CarmaLifecycleNode

```c++
#include <carma_ros2_utils/carma_lifecycle_node.hpp>

// Node extends the CarmaLifecycleNode
class CarmaLifecycleNodeTest : public carma_ros2_utils::CarmaLifecycleNode
{
public:
  CarmaLifecycleNodeTest(const rclcpp::NodeOptions &options)
      : CarmaLifecycleNode(options) {}

  ~CarmaLifecycleNodeTest() {};

  // Override the required transition handler to load configurations
  carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(get_logger(), "CARMA Lifecycle Test node is Configured!");

    // Create subscriptions
    system_alert_sub_ = create_subscription<carma_msgs::msg::SystemAlert>(
          "/some_topic", 100,
          [](auto) {}, this, std::placeholders::_1));

    // You would also create parameters/services/timers etc. here

    return CallbackReturn::SUCCESS;
  }

  private:
    rclcpp::Subscription<carma_msgs::msg::SystemAlert>::SharedPtr system_alert_sub_;
  // For this example there is no reason to override the other transition handlers
};
```

For some nodes you may want to user parameter callbacks to enable dynamic reconfiguring of the node.
The process for doing this can be a bit tedious so a helper method ```update_params``` with 2 overloads is provided to make it easier.
The following is an example of how to do this.

```c++
#include <carma_ros2_utils/carma_lifecycle_node.hpp>

class CarmaLifecycleNodeTest : public carma_ros2_utils::CarmaLifecycleNode
{
private:
  double my_private_param_ = 1.0; // Parameter behind set/get methods to update

public:

  int my_param_ = 0; // A public parameter to update

  // Method takes the new param value and returns the old one
  double set_my_private_param(double new_val) {
    double old = my_private_param_;
    my_private_param_ = new_val;
    return old;
  }

  CarmaLifecycleNodeTest(const rclcpp::NodeOptions &options)
      : CarmaLifecycleNode(options) {}

  ~CarmaLifecycleNodeTest() {};

  // When configuring we want to set up our parameter callback
  carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State & /*state*/) override
  {

    // Initial declaration and load of the parameters
    my_param_ = this->declare_parameter<int>("my_param", my_param_);
    my_private_param_ = this->declare_parameter<int>("my_private_param", my_private_param_);

    // Create parameter callbacks for runtime updates
    add_on_set_parameters_callback(
      [this](auto param_vec) {
          
          // update_params is called for an int parameter (more than one can be provided)
          // A pair of the parameter name and a reference to that parameter's variable are provided
          auto error = update_params<int>({ {"my_param", my_param_} }, param_vec);

          // Example of using the function callback. Lambdas are also supported
          auto error_2 = update_params<double>({ {"my_private_param", std::bind(&CarmaLifecycleNodeTest::set_my_private_param, this, std::placeholders::_1 )} }, param_vec);

          rcl_interfaces::msg::SetParametersResult result;
          result.successful = !error && !error_2;

          if (error) { // If the parameter could not be set and error is returned
              result.reason = error.get();
          }

          return result;
      });

    return CallbackReturn::SUCCESS;
  }

  private:
    rclcpp::Subscription<carma_msgs::msg::SystemAlert>::SharedPtr system_alert_sub_;
};
```

## Lifecycle Component Wrapper

The lifecycle component wrapper provides a mechanism for launch time wrapping of component rclcpp::Node to make them adhere to the ROS2 Lifecycle node state machine in a minimal way. A ROS2 component node can be thought of as having 2 possible overarching states: Loaded and Unloaded This means that they can be conceptually mapped onto the Lifecycle node state machine. Specifically, nodes can be loaded on activation and unloaded on deactivation/error/shutdown. While loading/unloading a node may incur a fair bit of overhead, it has the benefit of ensuring to the user that the non-lifecycle node will not actively interfere with other nodes in the system unless in the ACTIVE state. Under such a system the only new requirement would be a node container capable of enforcing this architecture. This would effectively be an alternative to the component_managers currently available in rclcpp. Then the user could wrap their external nodes with the existing launch commands used for component loading. The LifecycleComponentWrapper class in this package is this alternative component manager. Examples of how to use it are shown below.

### Example

```python
def generate_launch_description():
  
    lifecycle_container = ComposableNodeContainer( # Standard setup for using ROS2 components
        package='rclcpp_components',
        name='lifecycle_component_wrapper', 
        executable='lifecycle_component_wrapper_mt', # Select a multi-threaded (_mt) or single threaded (_st) wrapper lifecycle executable
        namespace="/",
        composable_node_descriptions=[
            ComposableNode( # This non-lifecycle node is now a exposed as a lifecycle node thanks to the wrapper
                package='cool_pkg',
                name='regular_node',
                plugin='cool_pkg_namespace::MyRegularNode',
            ),
        ]
    )

    return LaunchDescription([
        lifecycle_container,
    ])
```

If the user wishes to include a lifecycle component and a non-lifecycle component in the same container (for performance), then they can use the extra arguments ```is_lifecycle_node```. This will internally cause an unchecked cast to ```rclcpp_lifecycle::LifecycleNode```. The ```configure()``` and ```activate()``` methods will then be called on this classes. Application of these arguments to classes which do not directly inherit from this type will lead to undefined behavior. So caution should be taken in their usage.

```python
def generate_launch_description():
  
    lifecycle_container = ComposableNodeContainer( 
        package='rclcpp_components',
        name='lifecycle_component_wrapper', 
        executable='lifecycle_component_wrapper_mt', 
        namespace="/",
        composable_node_descriptions=[
            ComposableNode( 
                package='cool_pkg',
                name='my_lifecycle_node',
                plugin='cool_pkg_namespace::MyLifecycleNode',
                extra_arguments=[
                    {'use_intra_process_comms': True},
                    # cool_pkg_namespace::MyLifecycleNode extends rclcpp_lifecycle::LifecycleNode and does not overload configure() or activate() so it is safe to make this call here.
                    {'is_lifecycle_node' : True } 
                ],
            ),
        ]
    )

    return LaunchDescription([
        lifecycle_container,
    ])
```
