Pure pursuit {#pure-pursuit}
=============


# Purpose / Use cases

Autoware.Auto requires the basic motion controller. Motion controller is necessary to follow the reference trajectory and keep the lane correctly.

The controller takes a reference trajectory and the current vehicle state (pose and velocity) as inputs. The main output is the control command for the vehicle interface.

Pure pursuit is a basic algorithm for the trajectory following and widely used in autonomous robot applications. Pure pursuit algorithm finds the adequate target point from the trajectory and computes the radius of curvature. The longitudinal acceleration is computed using velocities of the current vehicle and target point.


# Design

The process of the pure pursuit algorithm is described below.
1. Compute errors between the current vehicle state and the target state. First, the target candidate is computed using the nearest neighbor search from the trajectory. And then interpolate the 1st and 2nd nearest neighbor points and compute the interpolated points based on the current vehicle position (line-point distance).
2. Check the updated trajectory and if its size is 0, the controller tries to do the emergency stop based on the given stop distance.
3. If the boolean for the delay compensation is true, compute the position difference between the pose timestamp and the computation timestamp using the current pose state
4. Compute the lookahead distance based on the current vehicle velocity and the conversion ratio from the speed to the distance.
5. Compute the target point from the trajectory based on the current position and the lookahead distance.
  - If the trajectory is updated, the start index for searching the target point is 0
  - If the trajectory is not updated, the start index for searching the target point is from the last target index
  - Searching the target point from the start index to the final index that satisfies following conditions. The first index that satisfies both conditions are selected as the target index. If there is no point that satisfies the second condition, the farthest index (final index) that satisfies the first condition is selected as the target point.
    1. The candidate target point is in the traveling direction of the vehicle
    2. The distance between the current vehicle and candidate target point is larger than the computed lookahead distance
6. If the interpolation mode is on, the interpolated point is computed using the target point and its previous point (index). The distance from the current vehicle position to the interpolated target point is equals to the lookahead distance.
7. Compute the curvature (radius) between the current position and the target point. The target velocity is extracted from the target point
8. The target velocity is extracted from the target point. If there is no target point, the emergency stop (acceleration / deceleration) is executed. If the car is stopped (under 0.001mps), do nothing.

## Updated things from the base paper

- Added the delay compensation using the CATR model
- Lookahead distance is estimated by the current vehicle velocity
- Added the traveling direction strategy to determine the target point. This enables the controller to take sequential pose inputs with the fixed trajectory.
- Added the target point interpolation to follow the lookahead distance

## Assumptions / Known limits

The coordinate information is like below. θ is the counterclockwise angular and its value range is [-pi, pi].
```
    x
    ^  
\ θ |
 \  |   
  \ |
<---+--- y  
```

The following limitations are present:

- This implementation assumes that the velocity's change is done smoothly. If the sign of the velocity is suddenly changed with large values like (30.0, 20.0, -10.0) (e.g. due to the unstable localization) and the trajectory is not updated, this implementation may not extract the target point from the trajectory because of the traveling direction strategy.
- If the difference of the timestamp between the pose and controller node is larger than 21 seconds, the delay compensation would be broken due to the int32's overflow.

## Inputs / Outputs / API

Inputs:
- `Trajectory.msg` is a sequence of trajectory points with the 2D position and pose
- `TrajectoryPointStamped.msg` is the current vehicle state (pose or velocity)

Outputs:
- `VehicleControlCommand.msg` is the control command for the vehicle interface
- `ControllerDiagnostic.msg` is the diagnosis result of the controller


## Configuration state

The following defines the configuration state:
- `minimum_lookahead_distance` (float32_t): The minimum lookahead distance (meter) for the pure pursuit. This prevents the vehicle from strong vibration
- `maximum_lookahead_distance` (float32_t): The maximum lookahead distance (meter) for the pure pursuit. This prevents the vehicle from strong vibration
- `speed_to_lookahead_ratio` (float32_t): The conversion ratio from the speed to the lookahead distance. The ratio is equal to the duration (s).
- `is_interpolate_lookahead_point` (bool8_t): Whether using the interpolation for determining the target position
- `is_delay_compensation` (bool8_t): Whether using the delay compensation for estimating the current vehicle position at the current timestamp
- `emergency_stop_distance` (float32_t): The distance for the emergency stop. This value is used when the controller has no reference trajectory, or when the trajectory is not updated and there is no frontward point in the traveling direction (180 degrees).
- `speed_thres_traveling_direction` (float32_t): The speed threshold for determining the traveling direction. If the absolute value of the vehicle velocity is larger than the threshold, the traveling direction is determined based on the sign of the velocity. If it is smaller than the threshold, the target point is determined by using the lookahead distance.

## Performance characterization

### Time

Searching the target point from the trajectory (length `n`) , which is `O(n)`, is the computational complexity of the pure pursuit algorithm.

### Space

The space complexity of the pure pursuit algorithm is dominated by the trajectory, which is `O(n)` in space.

# Security considerations

TBD by a security specialist.


# References / External links

- [Original paper](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf)
- [Autoware.AI implementation](https://github.com/CPFL/Autoware/tree/master/ros/src/computing/planning/motion/packages/waypoint_follower/nodes/pure_pursuit)


# Future extensions / Unimplemented parts



# Related issues

- #36: Export to open source
