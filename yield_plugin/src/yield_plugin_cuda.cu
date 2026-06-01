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
 * For the overlapping time window [t_lo, t_hi] the two endpoints move
 * linearly.  Relative displacement is therefore also linear:
 *
 *   r(s) = (dx0 + dvx*s, dy0 + dvy*s),  s ∈ [0, T=t_hi-t_lo]
 *
 * Squared distance is a convex quadratic in s; its minimum is found
 * analytically and clamped to [0, T].  A collision is recorded when the
 * minimum squared distance is ≤ collision_radius².
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
  const float x1a = ego_x[ego_i],     y1a = ego_y[ego_i],     t1a = ego_t[ego_i];
  const float x1b = ego_x[ego_i + 1], y1b = ego_y[ego_i + 1], t1b = ego_t[ego_i + 1];

  // Obstacle segment endpoints
  const float x2a = obs_x[base + obs_j],     y2a = obs_y[base + obs_j],     t2a = obs_t[base + obs_j];
  const float x2b = obs_x[base + obs_j + 1], y2b = obs_y[base + obs_j + 1], t2b = obs_t[base + obs_j + 1];

  // Temporal overlap
  const float t_lo = fmaxf(t1a, t2a);
  const float t_hi = fminf(t1b, t2b);
  if (t_lo >= t_hi) return;

  const float dt1 = t1b - t1a;
  const float dt2 = t2b - t2a;
  if (dt1 < 1e-6f || dt2 < 1e-6f) return;

  // Velocities
  const float vex = (x1b - x1a) / dt1;
  const float vey = (y1b - y1a) / dt1;
  const float vox = (x2b - x2a) / dt2;
  const float voy = (y2b - y2a) / dt2;

  // Positions at t_lo
  const float a1  = (t_lo - t1a) / dt1;
  const float ex0 = x1a + a1 * (x1b - x1a);
  const float ey0 = y1a + a1 * (y1b - y1a);

  const float a2  = (t_lo - t2a) / dt2;
  const float ox0 = x2a + a2 * (x2b - x2a);
  const float oy0 = y2a + a2 * (y2b - y2a);

  // Relative position at t_lo and relative velocity
  const float dx0 = ex0 - ox0;
  const float dy0 = ey0 - oy0;
  const float dvx = vex - vox;
  const float dvy = vey - voy;
  const float T   = t_hi - t_lo;

  // Minimise dist²(s) = (dx0+dvx*s)² + (dy0+dvy*s)²  for s ∈ [0, T]
  // d/ds = 0  →  s* = -(dx0*dvx + dy0*dvy) / (dvx²+dvy²)
  const float dv2 = dvx * dvx + dvy * dvy;
  float s_star;
  if (dv2 < 1e-10f) {
    s_star = 0.0f;
  } else {
    s_star = -(dx0 * dvx + dy0 * dvy) / dv2;
    s_star  = fmaxf(0.0f, fminf(T, s_star));
  }

  const float dx      = dx0 + dvx * s_star;
  const float dy      = dy0 + dvy * s_star;
  const float dist_sq = dx * dx + dy * dy;

  if (dist_sq <= collision_radius_sq) {
    atomic_min_f(&out_collision_t[obj], t_lo + s_star);
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

  // Flatten SoA for ego
  std::vector<float> h_ego_x(n_ego), h_ego_y(n_ego), h_ego_t(n_ego);
  for (int i = 0; i < n_ego; ++i) {
    h_ego_x[i] = ego_pts[i].x;
    h_ego_y[i] = ego_pts[i].y;
    h_ego_t[i] = ego_pts[i].t;
  }

  // Flatten SoA for obstacles
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

}  // namespace yield_plugin
