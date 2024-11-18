#!/usr/bin/env python3

import psutil
import subprocess
import time
import csv
import os
import argparse
from datetime import datetime


def parse_args():
    parser = argparse.ArgumentParser(description="Monitor CPU usage of ROS2 nodes")
    parser.add_argument(
        "--output-dir",
        "-o",
        default="carma-cpu-usage-logs",
        help="Directory to store output files (default: carma-cpu-usage-logs)",
    )
    return parser.parse_args()


def setup_logging_directory(output_dir):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    timestamp = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
    filename = f"cpu_usage_ros2_nodes_{timestamp}.csv"

    return os.path.join(output_dir, filename)


def main():
    args = parse_args()
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
            ]
        )
        print(f"Starting to monitor the CPU usage data and saving to: {output_file}")

        while True:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            total_cpu_percent = psutil.cpu_percent(interval=None)

            for proc in psutil.process_iter(
                ["pid", "name", "cpu_percent", "memory_percent", "cmdline"]
            ):
                try:
                    cmdline = " ".join(proc.info["cmdline"])
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

                except (
                    psutil.NoSuchProcess,
                    psutil.AccessDenied,
                    psutil.ZombieProcess,
                ):
                    continue

            time.sleep(1)


if __name__ == "__main__":
    main()
