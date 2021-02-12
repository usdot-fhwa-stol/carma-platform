#!/usr/bin/python

import sys
import csv
from bisect import bisect_left 
from enum import Enum
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import rosbag # Imported to python env with pip install --extra-index-url https://rospypi.github.io/simple/ rospy rosbag


# Usage
# process_traj_logs.py <file name>

if len(sys.argv) < 2:
  print("Need 1 arguments: process_bag.py <file name> ")
  exit()

print("Starting To Process Bag")
bag = rosbag.Bag(sys.argv[1])

plan_trajectory_time_steps = []
for topic, msg, t in bag.read_messages(topics=['/guidance/plan_trajectory']):
  # Create data to print for Plan Delegator -> Traj Executor
  plan_trajectory_time_steps.append([])
  for point in msg.trajectory_points:
    plan_trajectory_time_steps[-1].append(point.target_time.to_sec())

pure_pursuit_plan_trajectory_time_steps = []
for topic, msg, t in bag.read_messages(topics=['/guidance/pure_pursuit/plan_trajectory']):
  # Create data to print for Traj Executor -> Pure Pursuit Wrapper
  pure_pursuit_plan_trajectory_time_steps.append([])
  for point in msg.trajectory_points:
    pure_pursuit_plan_trajectory_time_steps[-1].append(point.target_time.to_sec())

carma_final_waypoints_times_steps = []
for topic, msg, t in bag.read_messages(topics=['/guidance/carma_final_waypoints']):
  # Create data to print for Pure Pursuit Wrapper -> Pure Pursuit
  carma_final_waypoints_times_steps.append([])
  for point in msg.waypoints:
    carma_final_waypoints_times_steps[-1].append(point.twist.twist.linear.x)

ctrl_raw = []
for topic, msg, t in bag.read_messages(topics=['/guidance/ctrl_raw']):
  # Create data to print for Pure Pursuit -> Twist Filter
  ctrl_raw.append(msg.cmd.linear_velocity)

ctrl_cmd = []
for topic, msg, t in bag.read_messages(topics=['/guidance/ctrl_cmd']):
  # Create data to print for Twist Filter -> Twist Gate 
  ctrl_cmd.append(msg.cmd.linear_velocity)

vehicle_cmd = []
for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle_cmd']):
  # Create data to print for Twist Gate -> SSC Interface (TODO double check)
  vehicle_cmd.append(msg.ctrl_cmd.linear_velocity)

print("Done Bag Processing")

print("Graphing Data")
# Takes list of dictionary
def index_plot_with_slider(figure_num, data, title, xlabel, ylabel):

  fig = plt.figure(figure_num)

  plt.title(title)
  plt.xlabel(xlabel)
  plt.ylabel(ylabel)
  time_step = data[0]
  l, = plt.plot(range(len(data[0])), data[0])

  time_step_ax = plt.axes([0.20, 0.01, 0.65, 0.03])
  time_step_sldr = Slider(time_step_ax, 'Time Step', 0.0, len(data) - 1.0, valinit=0, valstep=1)

  def update_timestep(val):
    time_step = data[int(time_step_sldr.val)]
    if (int(time_step_sldr.val) < 66 and int(time_step_sldr.val) > 62):
      print ("Trajectory Size: " + str(len(time_step)))
    l.set_xdata(range(len(time_step)))
    l.set_ydata(time_step)
    fig.canvas.draw_idle()

  time_step_sldr.on_changed(update_timestep)

  return (fig, l, time_step_sldr)

plot1= index_plot_with_slider(1, plan_trajectory_time_steps, 
  "/guidance/plan_trajectory", "Index", "Time (s)")

plot2= index_plot_with_slider(2, pure_pursuit_plan_trajectory_time_steps, 
  "/guidance/pure_pursuit/plan_trajectory", "Index", "Time (s)")

plot3= index_plot_with_slider(3, carma_final_waypoints_times_steps, 
  "/guidance/carma_final_waypoints", "Index", "Velocity (m/s)")

plt.show()