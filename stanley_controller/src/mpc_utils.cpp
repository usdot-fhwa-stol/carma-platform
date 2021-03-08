/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "stanley_controller/mpc_utils.h"

#include <ros/console.h>
#include <limits>
#include <algorithm>
#include <vector>

void MPCUtils::convertEulerAngleToMonotonic(std::vector<double>* const a)
{
  for (unsigned int i = 1; i < a->size(); ++i)
  {
    const double da = (*a)[i] - (*a)[i - 1];
    (*a)[i] = (*a)[i - 1] + amathutils::normalizeRadian(da);
  }
}

template <typename T1, typename T2>
bool MPCUtils::interp1d(const T1& index, const T2& values, const double& ref, double* const ret)
{
  *ret = 0.0;
  if (!(static_cast<int>(index.size()) == static_cast<int>(values.size())))
  {
    ROS_DEBUG("index and values must have same size, return false. size : idx = %d, values = %d",
             static_cast<int>(index.size()), static_cast<int>(values.size()));
    return false;
  }
  if (index.size() == 1)
  {
    ROS_DEBUG("index size is 1, too short. return false.");
    return false;
  }
  unsigned int end = index.size() - 1;
  if (ref < index[0])
  {
    *ret = values[0];
    return true;
  }
  if (index[end] < ref)
  {
    *ret = values[end];
    return true;
  }

  for (unsigned int i = 1; i < index.size(); ++i)
  {
    if (!(index[i] > index[i - 1]))
    {
      ROS_DEBUG("index must be monotonically increasing, return false. index[%d] = %f, but index[%d] = %f", i,
               index[i], i - 1, index[i - 1]);
      return false;
    }
  }
  unsigned int i = 1;
  while (ref > index[i])
  {
    ++i;
  }
  const double a = ref - index[i - 1];
  const double d_index = index[i] - index[i - 1];
  *ret = ((d_index - a) * values[i - 1] + a * values[i]) / d_index;
  return true;
}

// 1D interpolation
bool MPCUtils::interp1dMPCTraj(const std::vector<double>& index, const MPCTrajectory& values,
                               const std::vector<double>& ref_time, MPCTrajectory* const ret)
{
  if (index.size() != values.size())
  {
    ROS_DEBUG("index and values must have same size, return false.");
    return false;
  }
  if (index.size() == 1)
  {
    ROS_DEBUG("index size is 1, too short. return false.");
    return false;
  }

  for (unsigned int i = 1; i < index.size(); ++i)
  {
    if (!(index[i] > index[i - 1]))
    {
      ROS_DEBUG("index must be monotonically increasing, return false. index[%d] = %f, but index[%d] = %f", i,
               index[i], i - 1, index[i - 1]);
      return false;
    }
  }

  for (unsigned int i = 1; i < ref_time.size(); ++i)
  {
    if (!(ref_time[i] > ref_time[i - 1]))
    {
      ROS_DEBUG("reference point must be monotonically increasing, return false. ref_time[%d] = %f, but ref_time[%d] = "
               "%f",
               i, ref_time[i], i - 1, ref_time[i - 1]);
      return false;
    }
  }

  ret->clear();
  unsigned int i = 1;
  for (unsigned int j = 0; j < ref_time.size(); ++j)
  {
    double a, d_index;
    if (ref_time[j] > index.back())
    {
      a = 1.0;
      d_index = 1.0;
      i = index.size() - 1;
    }
    else if (ref_time[j] < index.front())
    {
      a = 0.0;
      d_index = 1.0;
      i = 1;
    }
    else
    {
      while (ref_time[j] > index[i])
      {
        ++i;
      }
      a = ref_time[j] - index[i - 1];
      d_index = index[i] - index[i - 1];
    }
    const double x = ((d_index - a) * values.x[i - 1] + a * values.x[i]) / d_index;
    const double y = ((d_index - a) * values.y[i - 1] + a * values.y[i]) / d_index;
    const double z = ((d_index - a) * values.z[i - 1] + a * values.z[i]) / d_index;
    const double yaw = ((d_index - a) * values.yaw[i - 1] + a * values.yaw[i]) / d_index;
    const double vx = ((d_index - a) * values.vx[i - 1] + a * values.vx[i]) / d_index;
    const double k = ((d_index - a) * values.k[i - 1] + a * values.k[i]) / d_index;
    const double t = ref_time[j];
    ret->push_back(x, y, z, yaw, vx, k, t);
  }
  return true;
}

void MPCUtils::calcTrajectoryYawFromXY(MPCTrajectory* const traj)
{
  if (traj->yaw.size() == 0)
    return;

  for (unsigned int i = 1; i < traj->yaw.size() - 1; ++i)
  {
    const double dx = traj->x[i + 1] - traj->x[i - 1];
    const double dy = traj->y[i + 1] - traj->y[i - 1];
    traj->yaw[i] = std::atan2(dy, dx);
  }
  if (traj->yaw.size() > 1)
  {
    traj->yaw[0] = traj->yaw[1];
    traj->yaw.back() = traj->yaw[traj->yaw.size() - 2];
  }
}

void MPCUtils::calcTrajectoryCurvature(const int curvature_smoothing_num, MPCTrajectory* const traj)
{
  unsigned int traj_k_size = traj->x.size();
  traj->k.clear();

  /* calculate curvature by circle fitting from three points */
  geometry_msgs::Point p1, p2, p3;
  for (unsigned int i = curvature_smoothing_num; i < traj_k_size - curvature_smoothing_num; ++i)
  {
    p1.x = traj->x[i - curvature_smoothing_num];
    p2.x = traj->x[i];
    p3.x = traj->x[i + curvature_smoothing_num];
    p1.y = traj->y[i - curvature_smoothing_num];
    p2.y = traj->y[i];
    p3.y = traj->y[i + curvature_smoothing_num];
    double den = std::max(amathutils::find_distance(p1, p2) * amathutils::find_distance(p2, p3) *
                              amathutils::find_distance(p3, p1),
                          0.0001);
    const double curvature = 2.0 * ((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)) / den;
    traj->k.push_back(curvature);
  }

  /* first and last curvature is copied from next value */
  for (int i = 0; i < curvature_smoothing_num; ++i)
  {
    traj->k.insert(traj->k.begin(), traj->k.front());
    traj->k.push_back(traj->k.back());
  }
}

void MPCUtils::convertWaypointsToMPCTraj(const autoware_msgs::Lane& lane, MPCTrajectory* const mpc_traj)
{
  mpc_traj->clear();
  const double k_tmp = 0.0;
  const double t_tmp = 0.0;
  for (const auto& wp : lane.waypoints)
  {
    const double x = wp.pose.pose.position.x;
    const double y = wp.pose.pose.position.y;
    const double z = wp.pose.pose.position.z;
    const double yaw = tf2::getYaw(wp.pose.pose.orientation);
    const double vx = wp.twist.twist.linear.x;
    mpc_traj->push_back(x, y, z, yaw, vx, k_tmp, t_tmp);
  }
}

void MPCUtils::convertWaypointsToMPCTrajWithDistanceResample(const autoware_msgs::Lane& path,
                                                             const std::vector<double>& path_time, const double& dl,
                                                             MPCTrajectory* const ref_traj)
{
  ref_traj->clear();
  double dist = 0.0;
  std::vector<double> dists;
  dists.push_back(0.0);

  for (int i = 1; i < static_cast<int>(path_time.size()); ++i)
  {
    double dx = path.waypoints.at(i).pose.pose.position.x - path.waypoints.at(i - 1).pose.pose.position.x;
    double dy = path.waypoints.at(i).pose.pose.position.y - path.waypoints.at(i - 1).pose.pose.position.y;
    dist += sqrt(dx * dx + dy * dy);
    dists.push_back(dist);
  }

  convertWaypointsToMPCTrajWithResample(path, path_time, dists, dl, ref_traj);
}

void MPCUtils::convertWaypointsToMPCTrajWithTimeResample(const autoware_msgs::Lane& path,
                                                         const std::vector<double>& path_time, const double& dt,
                                                         MPCTrajectory* const ref_traj)
{
  ref_traj->clear();
  convertWaypointsToMPCTrajWithResample(path, path_time, path_time, dt, ref_traj);
}

void MPCUtils::convertWaypointsToMPCTrajWithResample(const autoware_msgs::Lane& path,
                                                     const std::vector<double>& path_time,
                                                     const std::vector<double>& ref_index, const double& d_ref_index,
                                                     MPCTrajectory* const ref_traj)
{
  if (ref_index.size() == 0)
  {
    return;
  }

  for (unsigned int i = 1; i < ref_index.size(); ++i)
  {
    if (ref_index[i] < ref_index[i - 1])
    {
      ROS_DEBUG("[convertWaypointsToMPCTrajWithResample] resampling index must be monotonically increasing. idx[%d] = "
                "%f, idx[%d+1] = %f",
                i, ref_index[i], i, ref_index[i + 1]);
      return;
    }
  }

  double point = ref_index[0];
  while (point < ref_index.back())
  {
    unsigned int j = 1;
    while (point > ref_index.at(j))
    {
      ++j;
    }

    const double a = point - ref_index.at(j - 1);
    const double ref_index_dist = ref_index.at(j) - ref_index.at(j - 1);
    const geometry_msgs::Pose pos0 = path.waypoints.at(j - 1).pose.pose;
    const geometry_msgs::Pose pos1 = path.waypoints.at(j).pose.pose;
    const geometry_msgs::Twist twist0 = path.waypoints.at(j - 1).twist.twist;
    const geometry_msgs::Twist twist1 = path.waypoints.at(j).twist.twist;
    const double x = ((ref_index_dist - a) * pos0.position.x + a * pos1.position.x) / ref_index_dist;
    const double y = ((ref_index_dist - a) * pos0.position.y + a * pos1.position.y) / ref_index_dist;
    const double z = ((ref_index_dist - a) * pos0.position.z + a * pos1.position.z) / ref_index_dist;

    /* for singular point of euler angle */
    const double yaw0 = tf2::getYaw(pos0.orientation);
    const double dyaw = amathutils::normalizeRadian(tf2::getYaw(pos1.orientation) - yaw0);
    const double yaw1 = yaw0 + dyaw;
    const double yaw = ((ref_index_dist - a) * yaw0 + a * yaw1) / ref_index_dist;
    const double vx = ((ref_index_dist - a) * twist0.linear.x + a * twist1.linear.x) / ref_index_dist;
    const double curvature_tmp = 0.0;
    const double t = ((ref_index_dist - a) * path_time.at(j - 1) + a * path_time.at(j)) / ref_index_dist;
    ref_traj->push_back(x, y, z, yaw, vx, curvature_tmp, t);
    point += d_ref_index;
  }
}

void MPCUtils::calcPathRelativeTime(const autoware_msgs::Lane& path, std::vector<double>* const path_time)
{
  double t = 0.0;
  path_time->clear();
  path_time->push_back(t);
  for (int i = 0; i < static_cast<int>(path.waypoints.size() - 1); ++i)
  {
    const double x0 = path.waypoints.at(i).pose.pose.position.x;
    const double y0 = path.waypoints.at(i).pose.pose.position.y;
    const double z0 = path.waypoints.at(i).pose.pose.position.z;
    const double x1 = path.waypoints.at(i + 1).pose.pose.position.x;
    const double y1 = path.waypoints.at(i + 1).pose.pose.position.y;
    const double z1 = path.waypoints.at(i + 1).pose.pose.position.z;
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double dz = z1 - z0;
    const double dist = sqrt(dx * dx + dy * dy + dz * dz);
    double v = std::max(std::fabs(path.waypoints.at(i).twist.twist.linear.x), 1.0);
    t += (dist / v);
    path_time->push_back(t);
  }
}

bool MPCUtils::calcNearestPose(const MPCTrajectory& traj, const geometry_msgs::Pose& self_pose,
                               geometry_msgs::Pose* nearest_pose, unsigned int* nearest_index, double* min_dist_error,
                               double* nearest_yaw_error, double* nearest_time)
{
  int nearest_index_tmp = -1;
  double min_dist_squared = std::numeric_limits<double>::max();
  *nearest_yaw_error = std::numeric_limits<double>::max();
  for (uint32_t i = 0; i < traj.size(); ++i)
  {
    const double dx = self_pose.position.x - traj.x[i];
    const double dy = self_pose.position.y - traj.y[i];
    const double dist_squared = dx * dx + dy * dy;

    /* ignore when yaw error is large, for crossing path */
    const double err_yaw = amathutils::normalizeRadian(tf2::getYaw(self_pose.orientation) - traj.yaw[i]);
    if (fabs(err_yaw) < (M_PI / 3.0))
    {
      if (dist_squared < min_dist_squared)
      {
        /* save nearest index */
        min_dist_squared = dist_squared;
        *nearest_yaw_error = err_yaw;
        nearest_index_tmp = i;
      }
    }
  }
  if (nearest_index_tmp == -1)
  {
    ROS_DEBUG("[calcNearestPose] yaw error is over PI/3 for all waypoints. no closest waypoint found.");
    return false;
  }

  *nearest_index = nearest_index_tmp;

  *min_dist_error = std::sqrt(min_dist_squared);
  *nearest_time = traj.relative_time[*nearest_index];
  nearest_pose->position.x = traj.x[*nearest_index];
  nearest_pose->position.y = traj.y[*nearest_index];
  nearest_pose->orientation = amathutils::getQuaternionFromYaw(traj.yaw[*nearest_index]);
  return true;
};

bool MPCUtils::calcNearestPoseInterp(const MPCTrajectory& traj, const geometry_msgs::Pose& self_pose,
                                     geometry_msgs::Pose* nearest_pose, unsigned int* nearest_index,
                                     double* min_dist_error, double* nearest_yaw_error, double* nearest_time)
{
  if (traj.size() == 0)
  {
    ROS_DEBUG("[calcNearestPoseInterp] trajectory size is zero");
    return false;
  }
  const double my_x = self_pose.position.x;
  const double my_y = self_pose.position.y;
  const double my_yaw = tf2::getYaw(self_pose.orientation);

  int nearest_index_tmp = -1;
  double min_dist_squared = std::numeric_limits<double>::max();
  for (uint32_t i = 0; i < traj.size(); ++i)
  {
    const double dx = my_x - traj.x[i];
    const double dy = my_y - traj.y[i];
    const double dist_squared = dx * dx + dy * dy;

    /* ignore when yaw error is large, for crossing path */
    const double err_yaw = amathutils::normalizeRadian(my_yaw - traj.yaw[i]);
    if (fabs(err_yaw) < (M_PI / 3.0))
    {
      if (dist_squared < min_dist_squared)
      {
        /* save nearest index */
        min_dist_squared = dist_squared;
        nearest_index_tmp = i;
      }
    }
  }
  if (nearest_index_tmp == -1)
  {
    ROS_DEBUG("[calcNearestPoseInterp] yaw error is over PI/3 for all waypoints. no closest waypoint found.");
    return false;
  }

  *nearest_index = nearest_index_tmp;

  if (traj.size() == 1)
  {
    nearest_pose->position.x = traj.x[*nearest_index];
    nearest_pose->position.y = traj.y[*nearest_index];
    tf2::Quaternion q;
    q.setRPY(0, 0, traj.yaw[*nearest_index]);
    nearest_pose->orientation = tf2::toMsg(q);
    *nearest_time = traj.relative_time[*nearest_index];
    *min_dist_error = std::sqrt(min_dist_squared);
    *nearest_yaw_error = amathutils::normalizeRadian(my_yaw - traj.yaw[*nearest_index]);
    return true;
  }

  /* get second nearest index = next to nearest_index */
  int second_nearest_index = 0;
  if (*nearest_index == traj.size() - 1)
  {
    second_nearest_index = *nearest_index - 1;
  }
  else if (*nearest_index == 0)
  {
    second_nearest_index = 1;
  }
  else
  {
    double dx1, dy1, dist_squared1, dx2, dy2, dist_squared2;
    dx1 = my_x - traj.x[*nearest_index + 1];
    dy1 = my_y - traj.y[*nearest_index + 1];
    dist_squared1 = dx1 * dx1 + dy1 * dy1;
    dx2 = my_x - traj.x[*nearest_index - 1];
    dy2 = my_y - traj.y[*nearest_index - 1];
    dist_squared2 = dx2 * dx2 + dy2 * dy2;
    if (dist_squared1 < dist_squared2)
      second_nearest_index = *nearest_index + 1;
    else
      second_nearest_index = *nearest_index - 1;
  }

  const double a_sq = min_dist_squared;

  /* distance between my position and second nearest position */
  const double dx2 = my_x - traj.x[second_nearest_index];
  const double dy2 = my_y - traj.y[second_nearest_index];
  const double b_sq = dx2 * dx2 + dy2 * dy2;

  /* distance between first and second nearest position */
  const double dx3 = traj.x[*nearest_index] - traj.x[second_nearest_index];
  const double dy3 = traj.y[*nearest_index] - traj.y[second_nearest_index];
  const double c_sq = dx3 * dx3 + dy3 * dy3;

  /* if distance between two points are too close */
  if (c_sq < 1.0E-5)
  {
    nearest_pose->position.x = traj.x[*nearest_index];
    nearest_pose->position.y = traj.y[*nearest_index];
    tf2::Quaternion q;
    q.setRPY(0, 0, traj.yaw[*nearest_index]);
    nearest_pose->orientation = tf2::toMsg(q);
    *nearest_time = traj.relative_time[*nearest_index];
    *min_dist_error = std::sqrt(min_dist_squared);
    *nearest_yaw_error = amathutils::normalizeRadian(my_yaw - traj.yaw[*nearest_index]);
    return true;
  }

  /* linear interpolation */
  const double alpha = 0.5 * (c_sq - a_sq + b_sq) / c_sq;
  nearest_pose->position.x = alpha * traj.x[*nearest_index] + (1 - alpha) * traj.x[second_nearest_index];
  nearest_pose->position.y = alpha * traj.y[*nearest_index] + (1 - alpha) * traj.y[second_nearest_index];
  double tmp_yaw_err = traj.yaw[*nearest_index] - traj.yaw[second_nearest_index];
  if (tmp_yaw_err > M_PI)
  {
    tmp_yaw_err -= 2.0 * M_PI;
  }
  else if (tmp_yaw_err < -M_PI)
  {
    tmp_yaw_err += 2.0 * M_PI;
  }
  const double nearest_yaw = traj.yaw[second_nearest_index] + alpha * tmp_yaw_err;
  tf2::Quaternion q;
  q.setRPY(0, 0, nearest_yaw);
  nearest_pose->orientation = tf2::toMsg(q);
  *nearest_time = alpha * traj.relative_time[*nearest_index] + (1 - alpha) * traj.relative_time[second_nearest_index];
  *min_dist_error = std::sqrt(b_sq - c_sq * alpha * alpha);
  *nearest_yaw_error = amathutils::normalizeRadian(my_yaw - nearest_yaw);
  return true;
};

bool MPCUtils::filt_vector(const int num, std::vector<double>* const u)
{
  if (static_cast<int>(u->size()) < num)
  {
    ROS_DEBUG("[MovingAverageFilter] vector size is lower than moving average number");
    return false;
  }
  std::vector<double> filtered_u(u->size());
  for (int i = 0; i < static_cast<int>(u->size()); ++i)
  {
    double tmp = 0.0;
    int num_tmp = 0;
    int count = 0;
    if (i - num < 0)
    {
      num_tmp = i;
    }
    else if (i + num > static_cast<int>(u->size()) - 1)
    {
      num_tmp = static_cast<int>(u->size()) - i - 1;
    }
    else
    {
      num_tmp = num;
    }

    for (int j = -num_tmp; j <= num_tmp; ++j)
    {
      tmp += (*u)[i + j];
      ++count;
    }
    filtered_u[i] = tmp / count;
  }
  u->swap(filtered_u);
  return true;
}
