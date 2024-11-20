#!/usr/bin/env python3
import psutil
import time
import csv
import os
import argparse
from datetime import datetime

# Define ROS-related keywords to filter processes
ROS_KEYWORDS = {
    "ros",
    "node",
    "rviz",
    "rqt",
    "/opt/ros/",  # ROS installation path
    "/opt/carma/",  # CARMA ROS installation path
    "roscore",
    "rosmaster",
    "roslaunch",
    "rostopic",
    "rosnode",
    "rosbag",
    "ros2",
    "ros1_bridge",
    "rmw",  # ROS middleware
    "fastrtps",
    "cyclonedds",
    "rclcpp",
    "rclpy",
    "noetic",
    "foxy",
    "humble",
}

# Define processes to exclude (to avoid false positives)
# NOTE: Detection of these keywords overwrites the ROS_KEYWORDS
EXCLUDE_KEYWORDS = {"code", "chrome", "firefox", "vscode", "gnome"}


def parse_args():
    parser = argparse.ArgumentParser(description="Monitor CPU usage of ROS2 nodes")
    parser.add_argument(
        "--output-dir",
        "-o",
        default="carma-cpu-usage-logs",
        help="Directory to store output files (default: carma-cpu-usage-logs)",
    )
    parser.add_argument(
        "--include-pattern",
        "-i",
        help="Additional comma-separated patterns to include in process filtering",
    )
    return parser.parse_args()


def setup_logging_directory(output_dir):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    timestamp = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
    filename = f"cpu_usage_ros2_nodes_{timestamp}.csv"
    return os.path.join(output_dir, filename)


def is_ros_related_process(proc_info, cmdline):
    """
    Check if a process is ROS-related based on name and command line
    """
    # Convert process information to lowercase for case-insensitive matching
    name_lower = proc_info["name"].lower()
    cmdline_lower = cmdline.lower()

    # Check exclusions first
    if any(excl in name_lower or excl in cmdline_lower for excl in EXCLUDE_KEYWORDS):
        return False

    # Check for ROS-related keywords in process name and command line
    return any(
        keyword in name_lower or keyword in cmdline_lower for keyword in ROS_KEYWORDS
    )


def get_process_environment(pid):
    """
    Try to get ROS-related environment variables for a process
    """
    try:
        proc = psutil.Process(pid)
        env = proc.environ()
        ros_env = {k: v for k, v in env.items() if "ROS" in k}
        return bool(ros_env)
    except (psutil.NoSuchProcess, psutil.AccessDenied):
        return False


def main():
    args = parse_args()

    # Add any additional include patterns from command line
    if args.include_pattern:
        additional_patterns = set(args.include_pattern.split(","))
        ROS_KEYWORDS.update(additional_patterns)

    output_file = setup_logging_directory(args.output_dir)

    with open(output_file, mode="a") as file:
        writer = csv.writer(file)
        writer.writerow(
            [
                "Timestamp",
                "PID",
                "Process Name",
                "CPU (%)",
                "Memory (%)",
                "Command Line",
                "Total CPU (%)",
                "Total CPU Num",
                "Total Memory (%)",
                "Total Memory (GB)",
            ]
        )

        print(f"Starting to monitor the CPU usage data and saving to: {output_file}")

        # Total CPU and memory size in GB doesn't change
        total_memory_gb = psutil.virtual_memory().total / (1024**3)
        total_cpus = os.cpu_count()

        while True:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            total_cpu_percent = psutil.cpu_percent(interval=None)
            total_memory_percent = (
                psutil.virtual_memory().percent
            )  # Get total memory usage

            for proc in psutil.process_iter(
                ["pid", "name", "cpu_percent", "memory_percent", "cmdline"]
            ):
                try:
                    cmdline = (
                        " ".join(proc.info["cmdline"]) if proc.info["cmdline"] else ""
                    )

                    if is_ros_related_process(proc.info, cmdline):
                        pid = proc.info["pid"]
                        name = proc.info["name"]
                        cpu_percent = proc.info["cpu_percent"]
                        memory_percent = proc.info["memory_percent"]

                        writer.writerow(
                            [
                                timestamp,
                                pid,
                                name,
                                cpu_percent,
                                memory_percent,
                                cmdline,
                                total_cpu_percent,
                                total_cpus,
                                total_memory_percent,
                                total_memory_gb,
                            ]
                        )

                except (
                    psutil.NoSuchProcess,
                    psutil.AccessDenied,
                    psutil.ZombieProcess,
                ):
                    continue

            time.sleep(1)


if __name__ == "__main__":
    main()
