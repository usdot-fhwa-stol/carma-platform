# Configuration

CARMA Platform's configuration comprises three components. The `/opt/carma` directory contains files that are
vehicle independent, such as maps and routes. The `carma-config` Docker containers provide runtime parameters that vary
among vehicle classes[^1]. The vehicle calibration directory contains calibration data that is unique to individual
vehicles.

[^1]: A vehicle class denotes the deployed-to vehicle make/model and sensor/actuator payload

## `/opt/carma` directory

Most of Platform's calibration and log files reside in the `/opt/carma` directory, which has the following
hierarchy:

- `/opt/carma/maps` contains point cloud data (PCD) and OpenStreetMap (OSM) map files
- `/opt/carma/routes` contains comma-separated values (CSV) files that define different driving routes
- `/opt/carma/logs` contains Platform-specific logs and ROS bag files
- `/opt/carma/.ros` contains general ROS logs and temporary files
- `/opt/carma/yolo` contains classification weights for the
  [You Only Look Once (YOLO)](https://pjreddie.com/darknet/yolo/) object detection system
- `/opt/carma/vehicle` is a symbolic link (symlink) to Platform's hardware calibration directory



## `carma-config` container

The `carma-config` Docker container houses Platform's runtime configurations including IP addresses, environment
variables, and required subsystems. Platform uses the `docker-compose.yml` file from the active container when
starting. The `carma-config` Docker images use the following naming conventions:

- development images: `usdotfhwastoldev/carma-config:develop-<vehicle-class>`
- production images: `usdotfhwastol/carma-config:<carma-version>-<vehicle-class>`

where `<carma-version>` is the version string for a specific CARMA release (_e.g._, `carma-system-4.4.3`), and
`<vehicle-class>` is the string identifier for a particular vehicle deployment (_e.g._, `ford-fusion-sehybrid-2019`).

Multiple `carma-config` containers can live on the host system simultaneously, and you set the active one by using the
`carma` CLI. See Docker Hub for current [development](https://hub.docker.com/r/usdotfhwastoldev/carma-config) and
[production](https://hub.docker.com/r/usdotfhwastol/carma-config) `carma-config` images.

## Calibration directory

The calibration directory contains information about each hardware component's physical properties, such as its
mounting position, orientation, and intrinsic parameters. This directory is unique to a specific vehicle and is
symbolically linked to the `/opt/carma/vehicle` directory. Platform's subsystems will search along this path when
looking for their calibration data.

The actual calibration directory can live anywhere on the host system, but the final linked path must be
`/opt/carma/vehicle/calibration`. You are free to use any filesystem hierarchy inside the `calibration` directory
assuming it is compatible with Platform's components. Here is an example that contains parameters for Platform's model
predictive control (MPC) and pure pursuit subsystems:

```
/opt/carma/vehicle/calibration
├── mpc_follower
│   └── calibration.yaml
└── pure_pursuit
    └── calibration.yaml
```

!!! note

    Unlike most of Platform's code and configurations, its calibration files are not publicly available.
