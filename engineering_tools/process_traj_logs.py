#!/usr/bin/python3

#  Copyright (C) 2021 LEIDOS.
# 
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
# 
#  http://www.apache.org/licenses/LICENSE-2.0
# 
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

import sys
import csv
from bisect import bisect_left 
from enum import Enum
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

def binarySearch(a, x): 
    i = bisect_left(a, x) 
    if i: 
        return (i-1) 
    else: 
        return -1
# Usage
# process_traj_logs.py <file name> <start time> <end_time>
#
# Read the file until start_time is found.
# Read the file until end_time is found
# Grab all lines between start and end time
# Print layered graphs of all trajectories between those times
if len(sys.argv) < 4:
  print("Need 3 arguments: process_traj_logs.py <file name> <start time> <end_time>")
  exit()

data = []
with open(sys.argv[1], 'r') as infile, \
     open(sys.argv[1] + ".clean", 'w') as outfile:
    data = infile.read()
    data = data.replace("\x1b[0m", "")
    data = data.replace("\x1b[32m", "")
    outfile.write(data)

core_data = {
  "times": [],
  "content": []
}
with open(sys.argv[1] + ".clean", 'r') as csv_file:
  traj_data = csv.reader(csv_file, delimiter='|')
  i = 0

  for row in traj_data:
    if len(row) < 4:
      continue
    core_data["times"].append(float(row[0].strip()))
    core_data["content"].append(row[3].strip())
    #print(row)
    i+=1
    #if i == 15: # TODO remove
    #  print(core_data)
    #
    #  break

# Grab requested section
start_index = max(binarySearch(core_data["times"], float(sys.argv[2])) - 1, 0)
end_index = min(binarySearch(core_data["times"], float(sys.argv[3])) + 1, len(core_data["times"]))

print("Start index: " + str(start_index))
print("End index: " + str(end_index))

core_data["times"] = core_data["times"][start_index:end_index]
core_data["content"] = core_data["content"][start_index:end_index]

# Identify required graphs
class DataSource(Enum):
  NONE = 0
  RAW_POINTS = 1,
  TIME_BOUND_POINTS = 2,
  BACK_AND_FRONT_POINTS = 3,
  SAMPLED_POINTS = 4,
  RAW_CURVATURES = 5,
  PROCESSED_CURVATURES = 6,
  CURVATURE_CONSTRAINED_SPEEDS = 7,
  FINAL_YAWS = 8,
  SPEED_LIMIT_CONSTRAINED_SPEEDS = 9,
  SPEED_OP_REVERSE_STEP = 10,
  SPEED_OP_FORWARD = 11,
  AFTER_SPEED_OP = 12,
  AFTER_AVERAGE = 13,
  AFTER_MIN_SPEED = 14,
  FINAL_TIMES = 15
  
data_source = DataSource.NONE

core_data["time_steps"] = []

for content in core_data["content"]:
  if data_source == DataSource.NONE and "VehicleState" in content:
    core_data["time_steps"].append({
      DataSource.RAW_POINTS : [],
      DataSource.TIME_BOUND_POINTS : [],
      DataSource.BACK_AND_FRONT_POINTS : [],
      DataSource.SAMPLED_POINTS : [],
      DataSource.RAW_CURVATURES : [],
      DataSource.PROCESSED_CURVATURES : [],
      DataSource.CURVATURE_CONSTRAINED_SPEEDS : [],
      DataSource.FINAL_YAWS : [],
      DataSource.SPEED_LIMIT_CONSTRAINED_SPEEDS : [],
      DataSource.SPEED_OP_REVERSE_STEP : [],
      DataSource.SPEED_OP_FORWARD : [],
      DataSource.AFTER_SPEED_OP : [],
      DataSource.AFTER_AVERAGE : [],
      DataSource.AFTER_MIN_SPEED : [],
      DataSource.FINAL_TIMES : []
    })

    data_source = DataSource.RAW_POINTS
 
  if data_source == DataSource.RAW_POINTS and "Point:" in content and "Speed:" in content:
    point_speed = content.split(':')
    point = point_speed[1].split('Speed')
    xy = point[0].split(',')
    x = float(xy[0])
    y = float(xy[1])
    core_data["time_steps"][-1][DataSource.RAW_POINTS].append((x,y))

  if data_source == DataSource.RAW_POINTS and "Got time_bound_points with size:" in content:
    data_source = DataSource.TIME_BOUND_POINTS
 
  if data_source == DataSource.TIME_BOUND_POINTS and "Point:" in content and "Speed:" in content:
    point_speed = content.split(':')
    point = point_speed[1].split('Speed')
    xy = point[0].split(',')
    x = float(xy[0])
    y = float(xy[1])
    core_data["time_steps"][-1][DataSource.TIME_BOUND_POINTS].append((x,y))

  if data_source == DataSource.TIME_BOUND_POINTS and "Got back_and_future points with size" in content:
    data_source = DataSource.BACK_AND_FRONT_POINTS
 
  if data_source == DataSource.BACK_AND_FRONT_POINTS and "Point:" in content and "Speed:" in content:
    point_speed = content.split(':')
    point = point_speed[1].split('Speed')
    xy = point[0].split(',')
    x = float(xy[0])
    y = float(xy[1])
    core_data["time_steps"][-1][DataSource.BACK_AND_FRONT_POINTS].append((x,y))

  if data_source == DataSource.BACK_AND_FRONT_POINTS and "Got sampled points with size:" in content:
    data_source = DataSource.SAMPLED_POINTS
 
  if data_source == DataSource.SAMPLED_POINTS and "," in content and not ":" in content:
    xy = content.split(',')
    x = float(xy[0])
    y = float(xy[1])
    core_data["time_steps"][-1][DataSource.SAMPLED_POINTS].append((x,y))
 
  if (False and data_source == DataSource.RAW_CURVATURES or data_source == DataSource.SAMPLED_POINTS) and "better_curvature[i]:" in content:
    data_source = DataSource.RAW_CURVATURES
    split = content.split(':')
    c = float(split[1])
    core_data["time_steps"][-1][DataSource.RAW_CURVATURES].append(c)

  if (data_source == DataSource.PROCESSED_CURVATURES or data_source == DataSource.SAMPLED_POINTS) and "curvatures[i]:" in content:
    data_source = DataSource.PROCESSED_CURVATURES
    split = content.split(':')
    c = float(split[1])
    core_data["time_steps"][-1][DataSource.PROCESSED_CURVATURES].append(c)
  
  if (data_source == DataSource.CURVATURE_CONSTRAINED_SPEEDS or data_source == DataSource.PROCESSED_CURVATURES) and "ideal_speeds:" in content:
    data_source = DataSource.CURVATURE_CONSTRAINED_SPEEDS
    split = content.split(':')
    c = float(split[1])
    core_data["time_steps"][-1][DataSource.CURVATURE_CONSTRAINED_SPEEDS].append(c)

  if (data_source == DataSource.FINAL_YAWS or data_source == DataSource.CURVATURE_CONSTRAINED_SPEEDS) and "final_yaw_values[i]:" in content:
    data_source = DataSource.FINAL_YAWS
    split = content.split(':')
    c = float(split[1])
    core_data["time_steps"][-1][DataSource.FINAL_YAWS].append(c)

  if (data_source == DataSource.SPEED_LIMIT_CONSTRAINED_SPEEDS or data_source == DataSource.FINAL_YAWS) and "constrained_speed_limits:" in content:
    data_source = DataSource.SPEED_LIMIT_CONSTRAINED_SPEEDS
    split = content.split(':')
    c = float(split[1])
    core_data["time_steps"][-1][DataSource.SPEED_LIMIT_CONSTRAINED_SPEEDS].append(c)

  if (data_source == DataSource.SPEED_OP_REVERSE_STEP or data_source == DataSource.SPEED_LIMIT_CONSTRAINED_SPEEDS) and "only_reverse[i]:" in content:
    data_source = DataSource.SPEED_OP_REVERSE_STEP
    split = content.split(':')
    c = float(split[1])
    core_data["time_steps"][-1][DataSource.SPEED_OP_REVERSE_STEP].append(c)
  
  if (data_source == DataSource.SPEED_OP_FORWARD or data_source == DataSource.SPEED_OP_REVERSE_STEP) and "after_forward[i]:" in content:
    data_source = DataSource.SPEED_OP_FORWARD
    split = content.split(':')
    c = float(split[1])
    core_data["time_steps"][-1][DataSource.SPEED_OP_FORWARD].append(c)

  if (data_source == DataSource.AFTER_SPEED_OP or data_source == DataSource.SPEED_OP_FORWARD) and "postAccel[i]:" in content:
    data_source = DataSource.AFTER_SPEED_OP
    split = content.split(':')
    c = float(split[1])
    core_data["time_steps"][-1][DataSource.AFTER_SPEED_OP].append(c)
  
  if (data_source == DataSource.AFTER_AVERAGE or data_source == DataSource.AFTER_SPEED_OP) and "post_average[i]:" in content:
    data_source = DataSource.AFTER_AVERAGE
    split = content.split(':')
    c = float(split[1])
    core_data["time_steps"][-1][DataSource.AFTER_AVERAGE].append(c)

  if (data_source == DataSource.AFTER_MIN_SPEED or data_source == DataSource.AFTER_AVERAGE) and "post_min_speed[i]:" in content:
    data_source = DataSource.AFTER_MIN_SPEED
    split = content.split(':')
    c = float(split[1])
    core_data["time_steps"][-1][DataSource.AFTER_MIN_SPEED].append(c)

  if (data_source == DataSource.FINAL_TIMES or data_source == DataSource.AFTER_MIN_SPEED) and "times[i]:" in content:
    data_source = DataSource.FINAL_TIMES
    split = content.split(':')
    c = float(split[1])
    core_data["time_steps"][-1][DataSource.FINAL_TIMES].append(c)
  
  if data_source == DataSource.FINAL_TIMES and not ("times[i]:" in content):
    data_source = DataSource.NONE
  

print("DONE PROCESSING FILE")
print("CREATING GRAPHS")

# Takes list of dictionary
def xy_scatter_with_slider(figure_num, data, key, title, xlabel, ylabel):

  fig = plt.figure(figure_num)

  plt.title(title)
  plt.xlabel(xlabel)
  plt.ylabel(ylabel)
  time_step = data[0]
  l, = plt.plot([xy[0] for xy in time_step[key]], [xy[1] for xy in time_step[key]], '.')

  time_step_ax = plt.axes([0.20, 0.001, 0.65, 0.03])
  time_step_sldr = Slider(time_step_ax, 'Time Step', 0.0, len(data) - 1.0, valinit=0, valstep=1)

  def update_timestep(val):
    time_step = data[int(time_step_sldr.val)]
    l.set_xdata([xy[0] for xy in time_step[key]])
    l.set_ydata([xy[1] for xy in time_step[key]])
    fig.canvas.draw_idle()

  time_step_sldr.on_changed(update_timestep)

  return (fig, l, time_step_sldr)

# Takes list of dictionary
def index_plot_with_slider(figure_num, data, key, title, xlabel, ylabel):

  fig = plt.figure(figure_num)

  plt.title(title)
  plt.xlabel(xlabel)
  plt.ylabel(ylabel)
  time_step = data[0]
  l, = plt.plot(range(len(time_step[key])), time_step[key])

  time_step_ax = plt.axes([0.20, 0.01, 0.65, 0.03])
  time_step_sldr = Slider(time_step_ax, 'Time Step', 0.0, len(data) - 1.0, valinit=0, valstep=1)

  def update_timestep(val):
    time_step = data[int(time_step_sldr.val)]
    l.set_xdata(range(len(time_step[key])))
    l.set_ydata(time_step[key])
    fig.canvas.draw_idle()

  time_step_sldr.on_changed(update_timestep)

  return (fig, l, time_step_sldr)

plot1= xy_scatter_with_slider(1, core_data["time_steps"], DataSource.RAW_POINTS, 
  "Raw Downsampled Points from Lanelet Centerlines", "X (m)", "Y (m)")

plot2= xy_scatter_with_slider(2, core_data["time_steps"], DataSource.TIME_BOUND_POINTS, 
  "Time Bound Points from Lanelet Centerlines", "X (m)", "Y (m)")

plot3= xy_scatter_with_slider(3, core_data["time_steps"], DataSource.BACK_AND_FRONT_POINTS, 
  "Back and front points from Lanelet Centerlines", "X (m)", "Y (m)")

plot4= xy_scatter_with_slider(4, core_data["time_steps"], DataSource.SAMPLED_POINTS, 
  "Sampled points from spline fitting", "X (m)", "Y (m)")

plot5 = index_plot_with_slider(5, core_data["time_steps"], DataSource.RAW_CURVATURES, 
  "Raw Curvatures", "Index", "Curvature (1/r) (m)")

plot6 = index_plot_with_slider(6, core_data["time_steps"], DataSource.PROCESSED_CURVATURES, 
  "Processed Curvatures", "Index", "Curvature (1/r) (m)")

plot7 = index_plot_with_slider(7, core_data["time_steps"], DataSource.CURVATURE_CONSTRAINED_SPEEDS, 
  "Curvature constrained speeds", "Index", "Velocity (m/s)")

plot8 = index_plot_with_slider(8, core_data["time_steps"], DataSource.FINAL_YAWS, 
  "Final Yaw values", "Index", "Yaw (rad)")

plot9 = index_plot_with_slider(9, core_data["time_steps"], DataSource.SPEED_LIMIT_CONSTRAINED_SPEEDS, 
  "Speed Limit Constrained Speeds", "Index", "Velocity (m/s)")

plot10 = index_plot_with_slider(10, core_data["time_steps"], DataSource.SPEED_OP_REVERSE_STEP, 
  "Speed Optimization Reverse Step", "Index", "Velocity (m/s)")

plot11 = index_plot_with_slider(11, core_data["time_steps"], DataSource.SPEED_OP_FORWARD, 
  "Speed Optimization Forward Step", "Index", "Velocity (m/s)")

plot12 = index_plot_with_slider(12, core_data["time_steps"], DataSource.AFTER_SPEED_OP, 
  "Speed Optimization Output", "Index", "Velocity (m/s)")

plot13 = index_plot_with_slider(13, core_data["time_steps"], DataSource.AFTER_AVERAGE, 
  "Speed after Moving Average", "Index", "Velocity (m/s)")

plot14 = index_plot_with_slider(14, core_data["time_steps"], DataSource.AFTER_MIN_SPEED, 
  "Speed after applying minimum speed (FINAL SPEED)", "Index", "Velocity (m/s)")

plot15 = index_plot_with_slider(15, core_data["time_steps"], DataSource.FINAL_TIMES, 
  "Final Times", "Index", "Seconds (s)")

plt.show()
'''
core_data["time_steps"].append({
      DataSource.RAW_POINTS : [],
      DataSource.TIME_BOUND_POINTS : [],
      DataSource.BACK_AND_FRONT_POINTS : [],
      DataSource.SAMPLED_POINTS : [],
      DataSource.RAW_CURVATURES : [],
      DataSource.PROCESSED_CURVATURES : [],
      DataSource.CURVATURE_CONSTRAINED_SPEEDS : [],
      DataSource.FINAL_YAWS : [],
      DataSource.SPEED_LIMIT_CONSTRAINED_SPEEDS : [],
      DataSource.SPEED_OP_REVERSE_STEP : [],
      DataSource.SPEED_OP_FORWARD : [],
      DataSource.AFTER_SPEED_OP : [],
      DataSource.AFTER_AVERAGE : [],
      DataSource.AFTER_MIN_SPEED : [],
      DataSource.FINAL_TIMES : []
    })
'''

# Proceeded by : VehicleState
# Point: Speed:
# 
# Proceeded by : Got time_bound_points with size:
# Point: Speed:

# Proceeded by : Got back_and_future points with size
# Point: Speed: 

# Proceeded by : Got sampled points with size
# (nothing ) -74.5649, 329.079

# Proceeded by : nothing
# better_curvature[i]:

# Proceeded by : nothing
# curvatures[i]:

# Proceeded by : nothing
# ideal_speeds[i]:

# Proceeded by : nothing
# final_yaw_values[i]:

# Proceeded by : nothing
# final_actual_speeds[i]:

# Proceeded by : nothing
# only_reverse[i]:

# Proceeded by : nothing
# after_forward[i]:

# Proceeded by : nothing
# postAccel[i]:

# Proceeded by : nothing
# post_average[i]:

# Proceeded by : nothing
# post_min_speed[i]:

# Proceeded by : nothing
# times[i]:

# Proceeded by : nothing
# times[i]:












#with open('demofile.txt', "r") as f:
#  read_data = f.readline()


#if __name__ == "__main__":
#    data = "My data read from the Web"
#    print(data)
#   modified_data = process_data(data)
#    print(modified_data)