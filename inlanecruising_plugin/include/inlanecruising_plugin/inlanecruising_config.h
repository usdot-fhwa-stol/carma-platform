#pragma once

struct InLaneCruisingPluginConfig {
  double trajectory_time_length = 6.0; // Trajectory length in seconds
  double curve_resample_step_size = 1.0; // resample step size in meters
  int downsample_ratio = 8.0; // downsample amount for lanelet centerlines
  double minimum_speed = 2.2352; // minimum allowable speed // TODO better solution for min speed is needed
  double max_accel = 1.5; // maximum allowable acceleration in m/s^2
  int lookahead_count = 8; // Number of points to look ahead for speed reductio. Should correspond to a distance of m = curve_resample_step_size*lookahead_count
  double lateral_accel_limit = 1.5; // m/s^2
  int moving_average_window_size = 5;
};