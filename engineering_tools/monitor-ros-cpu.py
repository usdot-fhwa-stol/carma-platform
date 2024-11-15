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
       ./carma start all

    5. The script will automatically monitor and log CPU/memory usage
       of all ROS2 nodes and related processes during CARMA operation

    6. To stop monitoring:
       - Press Ctrl+C in the monitoring terminal
       - The CSV output file will be saved in the current directory as:
         'cpu_usage_ros2_nodes.csv' or 'cpu_usage_ros2_nodes-N.csv'

Output:
    - CSV file containing timestamp, process info, CPU and memory usage
    - Data can be used to analyze CARMA Platform resource utilization
"""

import psutil
import subprocess
import time
import csv
import os
from datetime import datetime


# Function to create a unique filename by appending a number
def get_unique_filename(base_filename):
    counter = 1
    filename, file_extension = os.path.splitext(base_filename)

    # Loop until a file with a unique name is found
    while os.path.isfile(base_filename):
        base_filename = f"{filename}-{counter}{file_extension}"
        counter += 1
    return base_filename


# Base path to save the CSV file
base_output_file = "cpu_usage_ros2_nodes.csv"

# Get a unique filename
output_file = get_unique_filename(base_output_file)

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
