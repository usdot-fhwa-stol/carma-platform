# Getting started

This tutorial describes the process for getting CARMA Platform installed and running on your machine.

!!! danger

    CARMA Platform's installation process is invasive in terms of user/group IDs, file permissions, and filesystem
    hierarchy. We therefore recommend deploying Platform to a dedicated machine.

## Prerequisites

### Software dependencies

This tutorial assumes your machine is running Ubuntu Desktop. You can install it by following the
[official tutorial][ubuntu_installation_tutorial]. Each Platform release targets a specific Ubuntu version, so be sure
to check that yours is compatible.

You will also need to install Docker Engine; see the [installation instructions][docker_install_guide] for help.

!!! note

    CARMA Platform requires a rootful Docker installation.

[ubuntu_installation_tutorial]: https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview
[docker_install_guide]: https://docs.docker.com/engine/install/ubuntu/

### User configuration

Platform creates a user called `carma` with user ID (UID) and group ID (GID) `1000` inside the Docker container. In
order for file permissions to work properly between the host machine and the container, the host user's UID and GID
need to also be set to `1000`:

```shell
sudo usermod --uid 1000 --gid 1000 carma
```

!!! note

    This command will fail if there is already a user with UID `1000`. Additionally, a group with GID `1000` must
    exist before running this command. See the `usermod` [man pages][usermod_manpages_url] for more information.

!!! warning

    The `usermod` command will update file ownership within the `carma` user's home directory, but it will not
    update ownership in other locations.

[usermod_manpages_url]: https://linux.die.net/man/8/usermod

## Install the command-line interface

The `carma` command-line interface (CLI) provides a convenient way to start and stop Platform and change its
configuration. You can install the CLI and the associated Bash completion functions from GitHub:

```shell
sudo curl -o /usr/local/bin/carma -L https://raw.githubusercontent.com/usdot-fhwa-stol/carma-platform/develop/engineering_tools/carma
sudo chmod 755 /usr/local/bin/carma
sudo curl -o /etc/bash_completion.d/__carma_autocomplete -L https://raw.githubusercontent.com/usdot-fhwa-stol/carma-platform/develop/engineering_tools/__carma_autocomplete
sudo chmod 644 /etc/bash_completion.d/__carma_autocomplete
```

## Set up the installation directory

Most of Platform's calibration and log files reside in the `/opt/carma` directory, which has the following
hierarchy:

- `/opt/carma/maps` contains point cloud data (PCD) and OpenStreetMap (OSM) map files
- `/opt/carma/routes` contains comma-separated values (CSV) files that define different driving routes
- `/opt/carma/logs` contains Platform-specific logs and ROS bag files
- `/opt/carma/.ros` contains general ROS logs and temporary files
- `/opt/carma/yolo` contains classification weights for the
  [You Only Look Once (YOLO)](https://pjreddie.com/darknet/yolo/) object detection system
- `/opt/carma/vehicle` is a symbolic link (symlink) to Platform's hardware calibration directory

!!! note

    Unlike most of Platform's code and configurations, its calibration files are not publicly available.

## Set the configuration version

Platform uses Docker images to deploy version-controlled configuration files. You can install and switch between
different configurations using the `carma` CLI. In this tutorial, we will use the `develop-development` configuration:

```shell
carma config install usdotfhwastoldev/carma-config:develop-development
carma config set usdotfhwastoldev/carma-config:develop-development
```
!!! note

    The [`usdotfhwastoldev`](https://hub.docker.com/u/usdotfhwastoldev) Docker Hub account contains Docker images for
    the latest development code, and the [`usdotfhwastol`](https://hub.docker.com/u/usdotfhwastol) account provides the
    latest release images.

## Start CARMA Platform

After following the above steps, you can start Platform with the following command:

```shell
carma start all
```

Open a browser and navigate to `localhost` to see CARMA's web UI.
