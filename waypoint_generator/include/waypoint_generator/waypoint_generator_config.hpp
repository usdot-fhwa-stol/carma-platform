#pragma once

struct WaypointGeneratorConfig {
  double _curvature_epsilon = 3.0;
  int _linearity_constraint = 2;
  double _lateral_accel_limit = 1.5;
  double _longitudinal_accel_limit = 1.5;
  double _longitudinal_decel_limit = 1.5;
  double _max_speed = 10.0;
  int _downsample_ratio = 2.0;
};