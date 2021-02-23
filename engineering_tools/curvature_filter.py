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
from bisect import bisect_left 

def binarySearch(a, x): 
    return bisect_left(a, x) 
    

def filter_curvatures(curvatures, downtrack_step_size):

  brackets = compute_curvature_brackets(2.5, 2.2352, 35.7632)
  #print("Brackets: " + str(brackets))
  c1 = constrain_to_brackets(brackets, curvatures, 1)
  c2 = denoise(c1, 4)
  return c2
  #return apply_curvature_rate_limits(c2, downtrack_step_size, 0.039)

def apply_curvature_rate_limits(curvatures, downtrack_step_size, max_curvature_rate):
  output = []
  if len(curvatures) == 0:
    return output

  output.append(curvatures[0])
  
  for i in range(1, len(curvatures)):
    delta_d = downtrack_step_size
    prev_curvature = output[-1]
    cur_curvature = curvatures[i]
    new_curvature = cur_curvature

    if cur_curvature > prev_curvature:
      # // Acceleration case
      limited_curvature = prev_curvature + (max_curvature_rate * delta_d)
      new_curvature = min(cur_curvature, limited_curvature)
    
    elif cur_curvature < prev_curvature:
      #  // Deceleration case
      limited_curvature = prev_curvature - (max_curvature_rate * delta_d)
      new_curvature = max(cur_curvature, limited_curvature)

    new_curvature = max(0.0, new_curvature)
    output.append(new_curvature)
    
  return output

def compute_curvature_brackets(acceleration_limit, bracket_size, max_value):

  curvature_bracket_upper_bounds = [sys.float_info.max]
  velocity = bracket_size # 2.2352 5mph as m/s

  while velocity < max_value:  # 35.7632 Less than 80mph
    curvature_bracket_upper_bounds.append(acceleration_limit / (velocity*velocity)) # curvature = a / v^2
    velocity += bracket_size
  
  curvature_bracket_upper_bounds.reverse()
  return curvature_bracket_upper_bounds

def constrain_to_brackets(brackets, values, round_direction=1):# -1 is down, 0 is rounding, 1 is round up
  output = []

  for val in values:
    index = binarySearch(brackets, val)
    if index == 0:
      output.append(brackets[0])
    elif index == len(brackets) - 1:
      output.append(brackets[-1])
    else:
      low_val = brackets[index - 1]
      high_val = brackets[index]
      if round_direction == -1:
        output.append(low_val)
      elif round_direction == 0: 
        halfway = ((high_val - low_val) / 2.0) + low_val
        if val < halfway:
            output.append(low_val); # Round down
        else:
            output.append(high_val); # Round up
      else:
        output.append(high_val)


  return output

def denoise(values, required_size):

  if len(values) == 0:
    print("Empty values")
    return []

  sections = []
  sections.append(
    [0, values[0], 1] # start index, value, element count
  )

  i = 0
  for v in values:
    if i == 0:
      i+=1
      continue

    prev_section_value = sections[-1][1]
    if v == prev_section_value:
      sections[-1][2] += 1
      continue
    
    sections.append(
      [i, v, 1] # start index, value, element count
    )
    i += 1

  if len(sections) == 1:
    return values # All values were the same

  #print("TimeStep: ")
  #print("Sections: " + str(sections))
  
  # TODO this loop might have a risk of pulling up the whole dataset
  # Do we want to give preference based on size? That runs the risk of pulling down the curve
  keep_looping = True
  while keep_looping:
    keep_looping = False
    i = 0
    for section in sections:
      if section[2] != 0 and section[2] < required_size:
        ref_sec = section
        if i == 0:
          ref_sec = sections[i+1]
        elif i == len(sections) - 1:
          ref_sec = sections[i-1]
        elif sections[i+1][1] >= sections[i-1][1]:
          ref_sec = sections[i+1]
        else:
          ref_sec = sections[i-1]
        
        print("Index: " + str(i))
        print("RefSecSize1: " + str(ref_sec[2]))
        ref_sec[2] += section[2] # Update the absorbing section count
        print("RefSecSize2: " + str(ref_sec[2])) # TODO This update process is causing a flop back and forth so this never exits
        section[2] = 0 # Mark this section as removed

        if ref_sec[2] < required_size:
          keep_looping = True
          break
      i+=1
  
  #print("After mod Sections: " + str(sections))
  output = []
  for section in sections:
    j = 0
    while j < section[2]:
      output.append(section[1])
      j += 1
#  print("Len Values: " + str(len(values)) + " Len Output: " + str(len(output)))
  return output



