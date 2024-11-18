#!/usr/bin/env python3

"""
CARMA Platform CPU Monitor Script

Requirements:
    - Python 3.6 or higher
    - psutil package (install with: pip3 install psutil)

Usage:
    1. Open a new terminal before starting CARMA Platform
    2. Navigate to the directory containing this script:
       cd /path/to/script/

    3. Run this script:
       python3 monitor-ros-cpu.py

    4. In a different terminal, start CARMA Platform:
       carma start all

    5. The script will automatically monitor and log CPU/memory usage
       of all ROS2 nodes and related processes during CARMA operation

    6. To stop monitoring:
       - Press Ctrl+C in the monitoring terminal
       - The CSV output file will be saved in the logs directory following ROS bag naming convention:
         'logs/cpu_usage_ros2_nodes_YYYY_MM_DD-HH_MM_SS.csv'

Output:
    - CSV file containing timestamp, process info, CPU and memory usage
    - Data can be used to analyze CARMA Platform resource utilization
"""

import psutil
import time
import csv
import os
from datetime import datetime


def setup_logging_directory():
    if not os.path.exists("logs"):
        os.makedirs("logs")

    timestamp = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
    filename = f"cpu_usage_ros2_nodes_{timestamp}.csv"

    return os.path.join("logs", filename)


output_file = setup_logging_directory()

# Open CSV file to store process usage data
with open(output_file, mode="a") as file:
    writer = csv.writer(file)

    # Write the header since it's a new file
    writer.writerow(
        [
            "Timestamp",
            "PID",
            "Process Name",
            "CPU (%)",
            "Memory (%)",
            "Command Line",
            "Total CPU (%)",
        ]
    )

    while True:
        # Get the current time with milliseconds
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

        # Get total CPU usage (overall system CPU usage)
        total_cpu_percent = psutil.cpu_percent(interval=None)  # Get immediate CPU usage

        # Get list of running processes
        for proc in psutil.process_iter(
            ["pid", "name", "cpu_percent", "memory_percent", "cmdline"]
        ):
            try:
                # Get CPU and memory usage of the process
                cmdline = " ".join(proc.info["cmdline"])  # Convert list to string
                if (
                    "ros" in proc.info["name"]
                    or "node" in proc.info["name"]
                    or "node" in cmdline
                    or "python3" in proc.info["name"]
                    or "ros" in cmdline
                ) and not ("code" in proc.info["name"]):
                    pid = proc.info["pid"]
                    name = proc.info["name"]
                    cpu_percent = proc.info["cpu_percent"]
                    memory_percent = proc.info["memory_percent"]

                    # Write process data to CSV, including the total CPU usage
                    writer.writerow(
                        [
                            timestamp,
                            pid,
                            name,
                            cpu_percent,
                            memory_percent,
                            cmdline,
                            total_cpu_percent,
                        ]
                    )

            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                # Handle any processes that may have terminated or restricted access
                continue

        # Sleep for 1 second before the next iteration
        time.sleep(1)
