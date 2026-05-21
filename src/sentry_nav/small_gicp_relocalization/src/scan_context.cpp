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

#include "small_gicp_relocalization/scan_context.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>

namespace small_gicp_relocalization
{

namespace
{

constexpr char kScanContextDbMagic[8] = {'S', 'C', 'D', 'B', 'v', '0', '0', '1'};
constexpr double kTwoPi = 2.0 * M_PI;
constexpr double kEmptyCellEpsilon = 1e-9;

template <typename T>
bool readPod(std::istream & is, T & out)
{
  is.read(reinterpret_cast<char *>(&out), sizeof(T));
  return static_cast<bool>(is);
}

template <typename T>
bool writePod(std::ostream & os, const T & in)
{
  os.write(reinterpret_cast<const char *>(&in), sizeof(T));
  return static_cast<bool>(os);
}

}  // namespace

ScanContextEngine::ScanContextEngine(const ScanContextConfig & cfg) : cfg_(cfg) {}

ScanContext ScanContextEngine::makeScanContext(
  const pcl::PointCloud<pcl::PointXYZ> & cloud_in_local) const
{
  ScanContext sc = ScanContext::Constant(cfg_.num_rings, cfg_.num_sectors, cfg_.height_min);

  const double max_radius = cfg_.max_radius;
  const double min_radius = 0.1;
  const int num_rings = cfg_.num_rings;
  const int num_sectors = cfg_.num_sectors;

  for (const auto & pt : cloud_in_local.points) {
    const double x = pt.x;
    const double y = pt.y;
    const double z = pt.z;

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }

    const double r = std::sqrt(x * x + y * y);
    if (r < min_radius || r > max_radius) {
      continue;
    }

    double phi = std::atan2(y, x);  // [-pi, pi]
    if (phi < -M_PI) {
      phi = -M_PI;
    } else if (phi >= M_PI) {
      phi = M_PI - kEmptyCellEpsilon;
    }

    int ring_idx = static_cast<int>(r / max_radius * num_rings);
    int sector_idx = static_cast<int>((phi + M_PI) / kTwoPi * num_sectors);
    ring_idx = std::clamp(ring_idx, 0, num_rings - 1);
    sector_idx = std::clamp(sector_idx, 0, num_sectors - 1);

    if (z > sc(ring_idx, sector_idx)) {
      sc(ring_idx, sector_idx) = z;
    }
  }

  return sc;
}

RingKey ScanContextEngine::makeRingKey(const ScanContext & sc) const
{
  const int num_rings = sc.rows();
  const int num_sectors = sc.cols();
  RingKey rk = RingKey::Zero(num_rings);

  if (num_sectors == 0) {
    return rk;
  }

  const double inv_sectors = 1.0 / static_cast<double>(num_sectors);
  for (int i = 0; i < num_rings; ++i) {
    int occupied = 0;
    for (int j = 0; j < num_sectors; ++j) {
      if (sc(i, j) > cfg_.height_min + kEmptyCellEpsilon) {
        ++occupied;
      }
    }
    rk(i) = static_cast<double>(occupied) * inv_sectors;
  }
  return rk;
}

std::pair<double, double> ScanContextEngine::distance(
  const ScanContext & sc_query, const ScanContext & sc_candidate) const
{
  if (sc_query.rows() != sc_candidate.rows() || sc_query.cols() != sc_candidate.cols()) {
    return {std::numeric_limits<double>::infinity(), 0.0};
  }

  const int num_rings = sc_query.rows();
  const int num_sectors = sc_query.cols();
  if (num_rings == 0 || num_sectors == 0) {
    return {std::numeric_limits<double>::infinity(), 0.0};
  }

  // Pre-compute per-column norms and emptiness for the candidate.
  std::vector<double> cand_norm(num_sectors, 0.0);
  std::vector<bool> cand_empty(num_sectors, true);
  const double empty_threshold = cfg_.height_min + kEmptyCellEpsilon;
  for (int j = 0; j < num_sectors; ++j) {
    double sum_sq = 0.0;
    bool empty = true;
    for (int i = 0; i < num_rings; ++i) {
      const double v = sc_candidate(i, j);
      sum_sq += v * v;
      if (v > empty_threshold) {
        empty = false;
      }
    }
    cand_norm[j] = std::sqrt(sum_sq);
    cand_empty[j] = empty;
  }

  // Pre-compute the same for the query (independent of shift).
  std::vector<double> query_norm(num_sectors, 0.0);
  std::vector<bool> query_empty(num_sectors, true);
  for (int j = 0; j < num_sectors; ++j) {
    double sum_sq = 0.0;
    bool empty = true;
    for (int i = 0; i < num_rings; ++i) {
      const double v = sc_query(i, j);
      sum_sq += v * v;
      if (v > empty_threshold) {
        empty = false;
      }
    }
    query_norm[j] = std::sqrt(sum_sq);
    query_empty[j] = empty;
  }

  double best_dist = std::numeric_limits<double>::infinity();
  int best_shift = 0;

  for (int shift = 0; shift < num_sectors; ++shift) {
    double sum_dist = 0.0;
    int valid_cols = 0;
    for (int j = 0; j < num_sectors; ++j) {
      const int qj = (j + shift) % num_sectors;
      // Skip pairs where both columns are empty: they carry no information
      // and would otherwise contribute a spurious zero distance.
      if (query_empty[qj] && cand_empty[j]) {
        continue;
      }
      double dot = 0.0;
      for (int i = 0; i < num_rings; ++i) {
        dot += sc_query(i, qj) * sc_candidate(i, j);
      }
      const double denom = query_norm[qj] * cand_norm[j] + kEmptyCellEpsilon;
      const double cosine = dot / denom;
      sum_dist += 1.0 - cosine;
      ++valid_cols;
    }
    if (valid_cols == 0) {
      continue;
    }
    const double avg_dist = sum_dist / static_cast<double>(valid_cols);
    if (avg_dist < best_dist) {
      best_dist = avg_dist;
      best_shift = shift;
    }
  }

  // Shift semantics (verified by /tmp/sc_smoke round-trip):
  //   query == R_z(yaw_shift) * candidate  (CCW positive, right-hand z-axis).
  // We compare query column (j+shift) % N with candidate column j. If query is
  // rotated CCW by Delta_theta relative to candidate (yaw_query = yaw_cand +
  // Delta_theta), every world feature's phi in the query local frame is
  // smaller by Delta_theta (because the local frame is what rotated, not the
  // features). To align the descriptors we need to rotate query columns to
  // higher sector indices by Delta_theta / sector_step, i.e.
  //   shift = +Delta_theta / sector_step (mod N).
  // Therefore yaw_shift = +shift * 2pi / N. Positive yaw_shift means query is
  // rotated CCW from candidate.
  double yaw_shift = static_cast<double>(best_shift) * kTwoPi / static_cast<double>(num_sectors);
  // Wrap to (-pi, pi].
  while (yaw_shift <= -M_PI) {
    yaw_shift += kTwoPi;
  }
  while (yaw_shift > M_PI) {
    yaw_shift -= kTwoPi;
  }
  return {best_dist, yaw_shift};
}

double ScanContextEngine::ringKeyDistance(const RingKey & rk_a, const RingKey & rk_b) const
{
  if (rk_a.size() != rk_b.size() || rk_a.size() == 0) {
    return std::numeric_limits<double>::infinity();
  }
  const double l1 = (rk_a - rk_b).cwiseAbs().sum();
  return l1 / static_cast<double>(rk_a.size());
}

bool ScanContextDB::save(const std::string & filepath) const
{
  if (poses.size() != descriptors.size() || poses.size() != ring_keys.size()) {
    std::cerr << "[ScanContextDB] save() failed: poses/descriptors/ring_keys size mismatch ("
              << poses.size() << " / " << descriptors.size() << " / " << ring_keys.size() << ")"
              << std::endl;
    return false;
  }

  std::ofstream os(filepath, std::ios::binary | std::ios::trunc);
  if (!os) {
    std::cerr << "[ScanContextDB] save() failed: cannot open '" << filepath << "' for writing"
              << std::endl;
    return false;
  }

  os.write(kScanContextDbMagic, sizeof(kScanContextDbMagic));

  const int32_t num_rings = static_cast<int32_t>(config.num_rings);
  const int32_t num_sectors = static_cast<int32_t>(config.num_sectors);
  if (!writePod(os, num_rings) || !writePod(os, num_sectors) ||
      !writePod(os, config.max_radius) || !writePod(os, config.height_min)) {
    std::cerr << "[ScanContextDB] save() failed: writing config" << std::endl;
    return false;
  }

  const int64_t count = static_cast<int64_t>(poses.size());
  if (!writePod(os, count)) {
    std::cerr << "[ScanContextDB] save() failed: writing count" << std::endl;
    return false;
  }

  const int sc_cells = config.num_rings * config.num_sectors;
  for (int64_t k = 0; k < count; ++k) {
    const Eigen::Vector3d & p = poses[k];
    const ScanContext & sc = descriptors[k];
    const RingKey & rk = ring_keys[k];

    if (sc.rows() != config.num_rings || sc.cols() != config.num_sectors) {
      std::cerr << "[ScanContextDB] save() failed: descriptor[" << k << "] shape ("
                << sc.rows() << "x" << sc.cols() << ") does not match config ("
                << config.num_rings << "x" << config.num_sectors << ")" << std::endl;
      return false;
    }
    if (rk.size() != config.num_rings) {
      std::cerr << "[ScanContextDB] save() failed: ring_key[" << k << "] size ("
                << rk.size() << ") does not match num_rings (" << config.num_rings << ")"
                << std::endl;
      return false;
    }

    const double pose[3] = {p.x(), p.y(), p.z()};
    os.write(reinterpret_cast<const char *>(pose), sizeof(pose));

    // Row-major dump of the scan context.
    std::vector<double> sc_buf(static_cast<size_t>(sc_cells));
    for (int i = 0; i < config.num_rings; ++i) {
      for (int j = 0; j < config.num_sectors; ++j) {
        sc_buf[static_cast<size_t>(i) * static_cast<size_t>(config.num_sectors) +
               static_cast<size_t>(j)] = sc(i, j);
      }
    }
    os.write(
      reinterpret_cast<const char *>(sc_buf.data()),
      static_cast<std::streamsize>(sc_buf.size() * sizeof(double)));

    std::vector<double> rk_buf(static_cast<size_t>(config.num_rings));
    for (int i = 0; i < config.num_rings; ++i) {
      rk_buf[static_cast<size_t>(i)] = rk(i);
    }
    os.write(
      reinterpret_cast<const char *>(rk_buf.data()),
      static_cast<std::streamsize>(rk_buf.size() * sizeof(double)));

    if (!os) {
      std::cerr << "[ScanContextDB] save() failed: writing entry " << k << std::endl;
      return false;
    }
  }

  return static_cast<bool>(os);
}

bool ScanContextDB::load(const std::string & filepath)
{
  std::ifstream is(filepath, std::ios::binary);
  if (!is) {
    std::cerr << "[ScanContextDB] load() failed: cannot open '" << filepath << "' for reading"
              << std::endl;
    return false;
  }

  char magic[8] = {0};
  is.read(magic, sizeof(magic));
  if (!is || std::memcmp(magic, kScanContextDbMagic, sizeof(kScanContextDbMagic)) != 0) {
    std::cerr << "[ScanContextDB] load() failed: bad magic (expected SCDBv001) in '" << filepath
              << "'" << std::endl;
    return false;
  }

  int32_t num_rings = 0;
  int32_t num_sectors = 0;
  double max_radius = 0.0;
  double height_min = 0.0;
  if (!readPod(is, num_rings) || !readPod(is, num_sectors) || !readPod(is, max_radius) ||
      !readPod(is, height_min)) {
    std::cerr << "[ScanContextDB] load() failed: reading config from '" << filepath << "'"
              << std::endl;
    return false;
  }
  if (num_rings <= 0 || num_sectors <= 0) {
    std::cerr << "[ScanContextDB] load() failed: invalid config (rings=" << num_rings
              << ", sectors=" << num_sectors << ")" << std::endl;
    return false;
  }

  ScanContextConfig loaded_cfg;
  loaded_cfg.num_rings = num_rings;
  loaded_cfg.num_sectors = num_sectors;
  loaded_cfg.max_radius = max_radius;
  loaded_cfg.height_min = height_min;

  int64_t count = 0;
  if (!readPod(is, count)) {
    std::cerr << "[ScanContextDB] load() failed: reading count" << std::endl;
    return false;
  }
  if (count < 0) {
    std::cerr << "[ScanContextDB] load() failed: negative count " << count << std::endl;
    return false;
  }

  std::vector<Eigen::Vector3d> loaded_poses;
  std::vector<ScanContext> loaded_descriptors;
  std::vector<RingKey> loaded_ring_keys;
  loaded_poses.reserve(static_cast<size_t>(count));
  loaded_descriptors.reserve(static_cast<size_t>(count));
  loaded_ring_keys.reserve(static_cast<size_t>(count));

  const size_t sc_cells = static_cast<size_t>(num_rings) * static_cast<size_t>(num_sectors);
  std::vector<double> sc_buf(sc_cells);
  std::vector<double> rk_buf(static_cast<size_t>(num_rings));

  for (int64_t k = 0; k < count; ++k) {
    double pose[3] = {0.0, 0.0, 0.0};
    is.read(reinterpret_cast<char *>(pose), sizeof(pose));
    if (!is) {
      std::cerr << "[ScanContextDB] load() failed: reading pose " << k << std::endl;
      return false;
    }

    is.read(
      reinterpret_cast<char *>(sc_buf.data()),
      static_cast<std::streamsize>(sc_buf.size() * sizeof(double)));
    if (!is) {
      std::cerr << "[ScanContextDB] load() failed: reading descriptor " << k << std::endl;
      return false;
    }

    is.read(
      reinterpret_cast<char *>(rk_buf.data()),
      static_cast<std::streamsize>(rk_buf.size() * sizeof(double)));
    if (!is) {
      std::cerr << "[ScanContextDB] load() failed: reading ring_key " << k << std::endl;
      return false;
    }

    ScanContext sc(num_rings, num_sectors);
    for (int i = 0; i < num_rings; ++i) {
      for (int j = 0; j < num_sectors; ++j) {
        sc(i, j) =
          sc_buf[static_cast<size_t>(i) * static_cast<size_t>(num_sectors) +
                 static_cast<size_t>(j)];
      }
    }
    RingKey rk(num_rings);
    for (int i = 0; i < num_rings; ++i) {
      rk(i) = rk_buf[static_cast<size_t>(i)];
    }

    loaded_poses.emplace_back(pose[0], pose[1], pose[2]);
    loaded_descriptors.emplace_back(std::move(sc));
    loaded_ring_keys.emplace_back(std::move(rk));
  }

  config = loaded_cfg;
  poses = std::move(loaded_poses);
  descriptors = std::move(loaded_descriptors);
  ring_keys = std::move(loaded_ring_keys);
  return true;
}

}  // namespace small_gicp_relocalization
