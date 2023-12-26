# hardcoded_params

This header only package stores constants that are meant to be hard-coded values in the CARMA Platform system. For example maximum allowed velocity or acceleration. Any package that needs such hard-coded limits should use the parameters in this package to ensure uniform settings across system components

## Usage

To use this package in your own package, simply include it as a catkin dependency and then access the required header file.

package.xml

```xml
  <depend>hardcoded_params</depend> <!-- build_depend should also be sufficient -->
```

CMakeList.txt file

```cmake
find_package(
  catkin REQUIRED COMPONENTS
    hardcoded_params
)

catkin_package(
  CATKIN_DEPENDS hardcoded_params
)
```

C++ Source Code

```c++
#include <hardcoded_params/control_limits/control_limits.h>

constexpr double my_max_val = hardcoded_params::control_limits::MAX_LONGITUDINAL_VELOCITY_MPS;

```
