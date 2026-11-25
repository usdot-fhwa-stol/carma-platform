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

#include "yield_plugin/yield_plugin_cuda.cuh"

#include <cuda_runtime.h>
#include <float.h>
#include <stdexcept>
#include <algorithm>
#include <string>

namespace yield_plugin
{

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

#define CUDA_CHECK(call)                                                      \
  do {                                                                        \
    cudaError_t _err = (call);                                                \
    if (_err != cudaSuccess) {                                                \
      throw std::runtime_error(                                               \
        std::string("CUDA error in " __FILE__ " line ") +                    \
        std::to_string(__LINE__) + ": " + cudaGetErrorString(_err));         \
    }                                                                         \
  } while (0)

// Atomic float minimum via CAS (CUDA lacks a native version for float).
__device__ static void atomic_min_f(float* __restrict__ addr, float val)
{
  int* addr_i = reinterpret_cast<int*>(addr);
  int  old    = *addr_i;
  int  assumed;
  do {
    assumed = old;
    if (__int_as_float(assumed) <= val) return;
    old = atomicCAS(addr_i, assumed, __float_as_int(val));
  } while (old != assumed);
}

// ---------------------------------------------------------------------------
// Kernel
// ---------------------------------------------------------------------------

/**
 * Each thread handles one (ego_segment i, obstacle_segment j) pair for one
 * object.
 *
 * Grid dims:
 *   x → ego segments   (blockIdx.x * blockDim.x + threadIdx.x)
 *   y → obs segments   (blockIdx.y * blockDim.y + threadIdx.y)
 *   z → object index   (blockIdx.z, one block layer per object)
 *
 * For the overlapping time window [overlap_start_t, overlap_end_t] the two
 * endpoints move linearly.  Relative displacement is therefore also linear:
 *
 *   relative_position(s) = (rel_pos_x + rel_vel_x*s, rel_pos_y + rel_vel_y*s),
 *   s ∈ [0, overlap_duration]
 *
 * Squared distance is a convex quadratic in s; its minimum is found
 * analytically and clamped to [0, overlap_duration].  A collision is recorded
 * when the minimum squared distance is ≤ collision_radius².
 *
 * The earliest collision time per object is stored via an atomic float min.
 */
__global__ void collision_kernel(
  const float* __restrict__ ego_x,
  const float* __restrict__ ego_y,
  const float* __restrict__ ego_t,
  int                        n_ego,
  const float* __restrict__ obs_x,
  const float* __restrict__ obs_y,
  const float* __restrict__ obs_t,
  const int*   __restrict__ obs_offsets,
  const int*   __restrict__ obs_sizes,
  float                      collision_radius_sq,
  int                        n_objects,
  float*       __restrict__ out_collision_t   // FLT_MAX = no collision
)
{
  const int ego_i = static_cast<int>(blockIdx.x) * blockDim.x + threadIdx.x;
  const int obs_j = static_cast<int>(blockIdx.y) * blockDim.y + threadIdx.y;
  const int obj   = static_cast<int>(blockIdx.z);

  if (obj >= n_objects)   return;
  if (ego_i >= n_ego - 1) return;

  const int n_obs = obs_sizes[obj];
  if (obs_j >= n_obs - 1) return;

  const int base = obs_offsets[obj];

  // Ego segment endpoints
  const float ego_seg_start_x = ego_x[ego_i],     ego_seg_start_y = ego_y[ego_i],     ego_seg_start_t = ego_t[ego_i];
  const float ego_seg_end_x   = ego_x[ego_i + 1], ego_seg_end_y   = ego_y[ego_i + 1], ego_seg_end_t   = ego_t[ego_i + 1];

  // Obstacle segment endpoints
  const float obj_seg_start_x = obs_x[base + obs_j],     obj_seg_start_y = obs_y[base + obs_j],     obj_seg_start_t = obs_t[base + obs_j];
  const float obj_seg_end_x   = obs_x[base + obs_j + 1], obj_seg_end_y   = obs_y[base + obs_j + 1], obj_seg_end_t   = obs_t[base + obs_j + 1];

  // Temporal overlap between the two segments
  const float overlap_start_t = fmaxf(ego_seg_start_t, obj_seg_start_t);
  const float overlap_end_t   = fminf(ego_seg_end_t, obj_seg_end_t);
  if (overlap_start_t >= overlap_end_t) return;

  const float ego_seg_duration = ego_seg_end_t - ego_seg_start_t;
  const float obj_seg_duration = obj_seg_end_t - obj_seg_start_t;
  if (ego_seg_duration < 1e-6f || obj_seg_duration < 1e-6f) return;

  // Velocities
  const float ego_vel_x = (ego_seg_end_x - ego_seg_start_x) / ego_seg_duration;
  const float ego_vel_y = (ego_seg_end_y - ego_seg_start_y) / ego_seg_duration;
  const float obj_vel_x = (obj_seg_end_x - obj_seg_start_x) / obj_seg_duration;
  const float obj_vel_y = (obj_seg_end_y - obj_seg_start_y) / obj_seg_duration;

  // Positions at the start of the overlap window
  const float ego_interp_ratio = (overlap_start_t - ego_seg_start_t) / ego_seg_duration;
  const float ego_pos_x = ego_seg_start_x + ego_interp_ratio * (ego_seg_end_x - ego_seg_start_x);
  const float ego_pos_y = ego_seg_start_y + ego_interp_ratio * (ego_seg_end_y - ego_seg_start_y);

  const float obj_interp_ratio = (overlap_start_t - obj_seg_start_t) / obj_seg_duration;
  const float obj_pos_x = obj_seg_start_x + obj_interp_ratio * (obj_seg_end_x - obj_seg_start_x);
  const float obj_pos_y = obj_seg_start_y + obj_interp_ratio * (obj_seg_end_y - obj_seg_start_y);

  // Relative position at the start of the overlap window, and relative velocity
  const float rel_pos_x = ego_pos_x - obj_pos_x;
  const float rel_pos_y = ego_pos_y - obj_pos_y;
  const float rel_vel_x = ego_vel_x - obj_vel_x;
  const float rel_vel_y = ego_vel_y - obj_vel_y;
  const float overlap_duration = overlap_end_t - overlap_start_t;

  // Minimise dist²(s) = (rel_pos_x + rel_vel_x*s)² + (rel_pos_y + rel_vel_y*s)²  for s ∈ [0, overlap_duration]
  // d/ds = 0  →  s* = -(rel_pos_x*rel_vel_x + rel_pos_y*rel_vel_y) / (rel_vel_x²+rel_vel_y²)
  const float rel_speed_sq = rel_vel_x * rel_vel_x + rel_vel_y * rel_vel_y;
  float closest_approach_offset_t;
  if (rel_speed_sq < 1e-10f) {
    closest_approach_offset_t = 0.0f;
  } else {
    closest_approach_offset_t = -(rel_pos_x * rel_vel_x + rel_pos_y * rel_vel_y) / rel_speed_sq;
    closest_approach_offset_t = fmaxf(0.0f, fminf(overlap_duration, closest_approach_offset_t));
  }

  const float closest_rel_x  = rel_pos_x + rel_vel_x * closest_approach_offset_t;
  const float closest_rel_y  = rel_pos_y + rel_vel_y * closest_approach_offset_t;
  const float closest_dist_sq = closest_rel_x * closest_rel_x + closest_rel_y * closest_rel_y;

  if (closest_dist_sq <= collision_radius_sq) {
    atomic_min_f(&out_collision_t[obj], overlap_start_t + closest_approach_offset_t);
  }
}

// ---------------------------------------------------------------------------
// Host wrapper
// ---------------------------------------------------------------------------

std::vector<CudaCollisionResult> cuda_check_all_collisions(
  const std::vector<CudaPoint>& ego_pts,
  const std::vector<CudaPoint>& obs_pts_flat,
  const std::vector<int>&       obs_offsets,
  const std::vector<int>&       obs_sizes,
  float                         collision_radius)
{
  const int n_ego    = static_cast<int>(ego_pts.size());
  const int n_objects = static_cast<int>(obs_sizes.size());

  std::vector<CudaCollisionResult> results(n_objects);

  if (n_ego < 2 || n_objects == 0 || obs_pts_flat.empty()) {
    return results;
  }

  const int total_obs    = static_cast<int>(obs_pts_flat.size());
  const int max_obs_size = *std::max_element(obs_sizes.begin(), obs_sizes.end());

  // Flatten SoA (structure of array) for ego
  std::vector<float> h_ego_x(n_ego), h_ego_y(n_ego), h_ego_t(n_ego);
  for (int i = 0; i < n_ego; ++i) {
    h_ego_x[i] = ego_pts[i].x;
    h_ego_y[i] = ego_pts[i].y;
    h_ego_t[i] = ego_pts[i].t;
  }

  // Flatten SoA (structure of array) for obstacles
  std::vector<float> h_obs_x(total_obs), h_obs_y(total_obs), h_obs_t(total_obs);
  for (int i = 0; i < total_obs; ++i) {
    h_obs_x[i] = obs_pts_flat[i].x;
    h_obs_y[i] = obs_pts_flat[i].y;
    h_obs_t[i] = obs_pts_flat[i].t;
  }

  // Output initialised to "no collision"
  std::vector<float> h_out_t(n_objects, FLT_MAX);

  // Device allocations
  float *d_ego_x{}, *d_ego_y{}, *d_ego_t{};
  float *d_obs_x{}, *d_obs_y{}, *d_obs_t{};
  int   *d_obs_offsets{}, *d_obs_sizes{};
  float *d_out_t{};

  auto free_all = [&]() noexcept {
    cudaFree(d_ego_x);   cudaFree(d_ego_y);   cudaFree(d_ego_t);
    cudaFree(d_obs_x);   cudaFree(d_obs_y);   cudaFree(d_obs_t);
    cudaFree(d_obs_offsets); cudaFree(d_obs_sizes);
    cudaFree(d_out_t);
  };

  try {
    CUDA_CHECK(cudaMalloc(&d_ego_x, n_ego    * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_ego_y, n_ego    * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_ego_t, n_ego    * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_obs_x, total_obs * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_obs_y, total_obs * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_obs_t, total_obs * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_obs_offsets, n_objects * sizeof(int)));
    CUDA_CHECK(cudaMalloc(&d_obs_sizes,   n_objects * sizeof(int)));
    CUDA_CHECK(cudaMalloc(&d_out_t,       n_objects * sizeof(float)));

    CUDA_CHECK(cudaMemcpy(d_ego_x, h_ego_x.data(), n_ego * sizeof(float), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_ego_y, h_ego_y.data(), n_ego * sizeof(float), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_ego_t, h_ego_t.data(), n_ego * sizeof(float), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_obs_x, h_obs_x.data(), total_obs * sizeof(float), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_obs_y, h_obs_y.data(), total_obs * sizeof(float), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_obs_t, h_obs_t.data(), total_obs * sizeof(float), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_obs_offsets, obs_offsets.data(), n_objects * sizeof(int), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_obs_sizes,   obs_sizes.data(),   n_objects * sizeof(int), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_out_t,  h_out_t.data(), n_objects * sizeof(float), cudaMemcpyHostToDevice));

    // 16×16 tile; z-dim = one slice per object
    constexpr int TILE = 16;
    const dim3 block(TILE, TILE, 1);
    const dim3 grid(
      static_cast<unsigned>((n_ego - 1   + TILE - 1) / TILE),
      static_cast<unsigned>((max_obs_size - 1 + TILE - 1) / TILE),
      static_cast<unsigned>(n_objects));

    collision_kernel<<<grid, block>>>(
      d_ego_x, d_ego_y, d_ego_t, n_ego,
      d_obs_x, d_obs_y, d_obs_t,
      d_obs_offsets, d_obs_sizes,
      collision_radius * collision_radius,
      n_objects,
      d_out_t);

    CUDA_CHECK(cudaGetLastError());
    CUDA_CHECK(cudaDeviceSynchronize());

    CUDA_CHECK(cudaMemcpy(h_out_t.data(), d_out_t, n_objects * sizeof(float), cudaMemcpyDeviceToHost));
  } catch (...) {
    free_all();
    throw;
  }
  free_all();

  for (int k = 0; k < n_objects; ++k) {
    if (h_out_t[k] < FLT_MAX) {
      results[k].has_collision    = true;
      results[k].collision_t_norm = h_out_t[k];
    }
  }
  return results;
}

bool cuda_is_available()
{
  int count = 0;
  cudaError_t err = cudaGetDeviceCount(&count);
  return (err == cudaSuccess && count > 0);
}

}  // namespace yield_plugin
