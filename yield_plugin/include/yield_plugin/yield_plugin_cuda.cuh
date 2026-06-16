/*
 * Copyright (C) 2026 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#pragma once

#include <vector>

namespace yield_plugin
{

// A single trajectory/prediction sample in flat SoA (structure of array) layout fed to the GPU.
// Timestamps are normalized (t - reference_t) before upload so float32
// precision is sufficient for the ~15-second planning horizon.
struct CudaPoint
{
  float x, y; // metres in a common Cartesian frame (e.g. ENU)
  float t;  // seconds, normalized relative to a caller-chosen reference
};

// Per-object result returned from the GPU collision batch.
struct CudaCollisionResult
{
  bool  has_collision        = false;
  float collision_t_norm     = 0.0f;  // collision time, same normalized basis as CudaPoint::t
};

/**
 * Batch-check all objects against the ego trajectory on the GPU.
 *
 * For every (ego_segment_i, obs_segment_j) pair with temporal overlap the
 * kernel analytically minimises the squared distance between the two linearly
 * moving points over the overlap interval and records a collision when the
 * minimum falls within collision_radius.  Each object is assigned one result
 * holding the *earliest* collision found across all segment pairs.
 *
 * @param ego_pts         Ego trajectory points, normalised timestamps.
 * @param obs_pts_flat    All objects' predicted states concatenated.
 * @param obs_offsets     obs_offsets[k] = start index in obs_pts_flat for object k.
 * @param obs_sizes       obs_sizes[k]   = number of states for object k.
 * @param collision_radius  Collision threshold in metres.
 * @return One CudaCollisionResult per object (same order as obs_offsets/obs_sizes).
 */
std::vector<CudaCollisionResult> cuda_check_all_collisions(
  const std::vector<CudaPoint>& ego_pts,
  const std::vector<CudaPoint>& obs_pts_flat,
  const std::vector<int>&       obs_offsets,
  const std::vector<int>&       obs_sizes,
  float                         collision_radius);

// Returns true when at least one CUDA device is reachable with a driver that
// matches the compiled runtime.  Safe to call from plain C++ translation units.
bool cuda_is_available();

}  // namespace yield_plugin
