// Copyright 2026 Boombroke
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SMALL_GICP_RELOCALIZATION__SCAN_CONTEXT_HPP_
#define SMALL_GICP_RELOCALIZATION__SCAN_CONTEXT_HPP_

#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace small_gicp_relocalization
{

/// Scan Context descriptor configuration.
///
/// All fields default to the values used by the Sentry26 relocalization stack.
/// `num_rings` and `num_sectors` define the polar grid resolution.
/// `max_radius` is the maximum point distance considered (meters); points
/// further out are dropped. `height_min` is the sentinel value used to fill
/// empty cells - it must be lower than any z value expected in the cloud,
/// because cells keep the maximum z of all points falling inside them.
struct ScanContextConfig
{
  int num_rings = 20;
  int num_sectors = 60;
  double max_radius = 30.0;
  double height_min = -2.0;
};

/// Scan Context descriptor: rings x sectors matrix of max-z values.
using ScanContext = Eigen::MatrixXd;

/// Ring key: per-ring occupancy ratio, used as a cheap pre-filter.
using RingKey = Eigen::VectorXd;

class ScanContextEngine
{
public:
  explicit ScanContextEngine(const ScanContextConfig & cfg = {});

  /// Build a Scan Context descriptor from a point cloud expressed in the
  /// sample point's local frame (origin = sample location). The descriptor is
  /// rotation invariant by construction (rotation around z maps to a column
  /// shift), so the caller does not need to align the cloud to a specific yaw.
  ScanContext makeScanContext(const pcl::PointCloud<pcl::PointXYZ> & cloud_in_local) const;

  /// Extract the ring key (occupancy ratio per ring) from a Scan Context.
  RingKey makeRingKey(const ScanContext & sc) const;

  /// Rotation-invariant Scan Context distance (column-shifted cosine
  /// distance). Returns {min_distance, yaw_shift_rad} where yaw_shift_rad is
  /// the rotation that aligns `sc_query` to `sc_candidate` (positive value
  /// means the query frame is rotated clockwise relative to the candidate by
  /// that angle).
  std::pair<double, double> distance(
    const ScanContext & sc_query, const ScanContext & sc_candidate) const;

  /// Ring-key L1 distance, normalized by the number of rings. Used to short
  /// list candidates before evaluating the full Scan Context distance.
  double ringKeyDistance(const RingKey & rk_a, const RingKey & rk_b) const;

  const ScanContextConfig & config() const { return cfg_; }

private:
  ScanContextConfig cfg_;
};

/// Persistent Scan Context database.
///
/// Contains the engine configuration plus per-sample (pose, descriptor,
/// ring_key) tuples. The on-disk layout uses a fixed magic header and the
/// configuration is validated on load - mismatched grids are rejected to
/// avoid silently producing meaningless distances.
struct ScanContextDB
{
  ScanContextConfig config;
  std::vector<Eigen::Vector3d> poses;     ///< (x, y, 0) sample poses in map frame
  std::vector<ScanContext> descriptors;   ///< Per-sample Scan Context
  std::vector<RingKey> ring_keys;         ///< Cached ring keys (one per descriptor)

  /// Serialize to the .scdb binary format. Returns false on I/O failure.
  bool save(const std::string & filepath) const;

  /// Load from a .scdb file produced by `save`. Returns false on I/O failure
  /// or magic/config mismatch.
  bool load(const std::string & filepath);
};

}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__SCAN_CONTEXT_HPP_
