#!/usr/bin/python

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
 
  if data_source == DataSource.SAMPLED_POINTS and "," in content:
    xy = content.split(',')
    x = float(xy[0])
    y = float(xy[1])
    core_data["time_steps"][-1][DataSource.SAMPLED_POINTS].append((x,y))
 
  if (data_source == DataSource.RAW_CURVATURES or data_source == DataSource.SAMPLED_POINTS) and "better_curvature[i]:" in content:
    data_source = DataSource.RAW_CURVATURES
    split = content.split(':')
    c = float(split[1])
    core_data["time_steps"][-1][DataSource.RAW_CURVATURES].append(c)

  if (data_source == DataSource.PROCESSED_CURVATURES or data_source == DataSource.RAW_CURVATURES) and "curvatures[i]:" in content:
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

  if (data_source == DataSource.SPEED_LIMIT_CONSTRAINED_SPEEDS or data_source == DataSource.FINAL_YAWS) and "final_actual_speeds:" in content:
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
  
plt.figure(0)
plt.scatter([xy[0] for xy in core_data["time_steps"][0][DataSource.RAW_POINTS]], [xy[1] for xy in core_data["time_steps"][0][DataSource.RAW_POINTS]])
plt.title("Raw Downsampled Points from Lanelet Centerlines")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")

plt.figure(1)
plt.scatter([xy[0] for xy in core_data["time_steps"][0][DataSource.TIME_BOUND_POINTS]], [xy[1] for xy in core_data["time_steps"][0][DataSource.TIME_BOUND_POINTS]])
plt.title("Time Bound Points from Lanelet Centerlines")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")

plt.figure(2)
plt.scatter([xy[0] for xy in core_data["time_steps"][0][DataSource.BACK_AND_FRONT_POINTS]], [xy[1] for xy in core_data["time_steps"][0][DataSource.BACK_AND_FRONT_POINTS]])
plt.title("Back and front points from Lanelet Centerlines")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")

plt.figure(3)
plt.scatter([xy[0] for xy in core_data["time_steps"][0][DataSource.SAMPLED_POINTS]], [xy[1] for xy in core_data["time_steps"][0][DataSource.SAMPLED_POINTS]])
plt.title("Sampled points for spline fitting")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")

plt.figure(5)
plt.plot(range(len(core_data["time_steps"][0][DataSource.RAW_CURVATURES])), core_data["time_steps"][0][DataSource.RAW_CURVATURES])
plt.title("Raw Curvatures")
plt.xlabel("Index")
plt.ylabel("Curvature (1/r) (m)")

plt.figure(6)
plt.plot(range(len(core_data["time_steps"][0][DataSource.PROCESSED_CURVATURES])), core_data["time_steps"][0][DataSource.PROCESSED_CURVATURES])
plt.title("Processed Curvatures")
plt.xlabel("Index")
plt.ylabel("Curvature (1/r) (m)")

plt.figure(7)
plt.plot(range(len(core_data["time_steps"][0][DataSource.CURVATURE_CONSTRAINED_SPEEDS])), core_data["time_steps"][0][DataSource.CURVATURE_CONSTRAINED_SPEEDS])
plt.title("Curvature constrained speeds")
plt.xlabel("Index")
plt.ylabel("Velocity (m/s)")

plt.figure(8)
plt.plot(range(len(core_data["time_steps"][0][DataSource.FINAL_YAWS])), core_data["time_steps"][0][DataSource.FINAL_YAWS])
plt.title("Final Yaw values")
plt.xlabel("Index")
plt.ylabel("Yaw (rad)")

plt.figure(9)
plt.plot(range(len(core_data["time_steps"][0][DataSource.SPEED_LIMIT_CONSTRAINED_SPEEDS])), core_data["time_steps"][0][DataSource.SPEED_LIMIT_CONSTRAINED_SPEEDS])
plt.title("Speed Limit Constrained Speeds")
plt.xlabel("Index")
plt.ylabel("Velocity (m/s)")

plt.figure(10)
plt.plot(range(len(core_data["time_steps"][0][DataSource.SPEED_OP_REVERSE_STEP])), core_data["time_steps"][0][DataSource.SPEED_OP_REVERSE_STEP])
plt.title("Speed Optimization Reverse Step")
plt.xlabel("Index")
plt.ylabel("Velocity (m/s)")

plt.figure(11)
plt.plot(range(len(core_data["time_steps"][0][DataSource.SPEED_OP_FORWARD])), core_data["time_steps"][0][DataSource.SPEED_OP_FORWARD])
plt.title("Speed Optimization Forward Step")
plt.xlabel("Index")
plt.ylabel("Velocity (m/s)")

plt.figure(12)
plt.plot(range(len(core_data["time_steps"][0][DataSource.AFTER_SPEED_OP])), core_data["time_steps"][0][DataSource.AFTER_SPEED_OP])
plt.title("Speed Optimization Output")
plt.xlabel("Index")
plt.ylabel("Velocity (m/s)")

plt.figure(13)
plt.plot(range(len(core_data["time_steps"][0][DataSource.AFTER_AVERAGE])), core_data["time_steps"][0][DataSource.AFTER_AVERAGE])
plt.title("Speed after Moving Average")
plt.xlabel("Index")
plt.ylabel("Velocity (m/s)")




final_speed_fig = plt.figure(14)

plt.title("Speed after applying minimum speed (FINAL SPEED)")
plt.xlabel("Index")
plt.ylabel("Velocity (m/s)")
l, = plt.plot(range(len(time_step[DataSource.AFTER_MIN_SPEED])), time_step[DataSource.AFTER_MIN_SPEED])

time_step_ax = plt.axes([0.20, 0.01, 0.65, 0.03])
time_step_sldr = Slider(time_step_ax, 'Time Step', 0.0, len(core_data["time_steps"]) - 1.0, valinit=0, valstep=1)

def update_final_speed_timestep(val):
  time_step = core_data["time_steps"][int(time_step_sldr.val)]
  l.set_xdata(range(len(time_step[DataSource.AFTER_MIN_SPEED])))
  l.set_ydata(time_step[DataSource.AFTER_MIN_SPEED])
  final_speed_fig.canvas.draw_idle()

time_step_sldr.on_changed(update_final_speed_timestep)

plt.figure(15)
plt.plot(range(len(core_data["time_steps"][0][DataSource.FINAL_TIMES])), core_data["time_steps"][0][DataSource.FINAL_TIMES])
plt.title("Final Times")
plt.xlabel("Index")
plt.ylabel("Seconds (s)")

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