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

// Offline tool: build a Scan Context descriptor database (.scdb) from a prior
// PCD map. The map is sampled on a regular grid; for each sample pose the
// local cloud (cropped by max_radius) is fed into ScanContextEngine and the
// resulting descriptor + ring_key + (x, y) pose are stored. The output file
// is consumed by the relocalization node at startup for global recovery.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "small_gicp_relocalization/scan_context.hpp"

namespace
{

struct CliArgs
{
  std::string pcd_path;
  std::string output_path;
  double grid_spacing = 1.5;
  double max_radius = 30.0;
  int num_rings = 20;
  int num_sectors = 60;
  double height_min = -2.0;
  int min_points = 200;
  bool show_help = false;
};

void printUsage(std::ostream & os, const char * argv0)
{
  os << "Usage: " << argv0 << " --pcd <path> --output <path> [options]\n"
     << "\n"
     << "Build a Scan Context descriptor database (.scdb) from a prior PCD map.\n"
     << "\n"
     << "Required:\n"
     << "  --pcd <path>          Input prior map (PCD, pcl::PointXYZ).\n"
     << "  --output <path>       Output .scdb file.\n"
     << "\n"
     << "Options:\n"
     << "  --grid-spacing <m>    Sampling grid spacing in meters (default: 1.5).\n"
     << "  --max-radius <m>      Scan Context max radius (default: 30.0).\n"
     << "  --num-rings <n>       Number of rings (default: 20).\n"
     << "  --num-sectors <n>     Number of sectors (default: 60).\n"
     << "  --height-min <m>      Empty-cell sentinel z (default: -2.0).\n"
     << "  --min-points <n>      Drop poses whose local cloud has fewer points\n"
     << "                        than this (default: 200).\n"
     << "  -h, --help            Show this message.\n";
}

bool parseDouble(const std::string & s, double & out)
{
  try {
    size_t idx = 0;
    out = std::stod(s, &idx);
    return idx == s.size();
  } catch (...) {
    return false;
  }
}

bool parseInt(const std::string & s, int & out)
{
  try {
    size_t idx = 0;
    out = std::stoi(s, &idx);
    return idx == s.size();
  } catch (...) {
    return false;
  }
}

bool parseArgs(int argc, char ** argv, CliArgs & args)
{
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (key == "-h" || key == "--help") {
      args.show_help = true;
      return true;
    }

    auto need_value = [&](const char * name) -> const char * {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for " << name << std::endl;
        return nullptr;
      }
      return argv[++i];
    };

    if (key == "--pcd") {
      const char * v = need_value("--pcd");
      if (!v) return false;
      args.pcd_path = v;
    } else if (key == "--output") {
      const char * v = need_value("--output");
      if (!v) return false;
      args.output_path = v;
    } else if (key == "--grid-spacing") {
      const char * v = need_value("--grid-spacing");
      if (!v || !parseDouble(v, args.grid_spacing) || args.grid_spacing <= 0.0) {
        std::cerr << "Invalid --grid-spacing (must be > 0)" << std::endl;
        return false;
      }
    } else if (key == "--max-radius") {
      const char * v = need_value("--max-radius");
      if (!v || !parseDouble(v, args.max_radius) || args.max_radius <= 0.0) {
        std::cerr << "Invalid --max-radius (must be > 0)" << std::endl;
        return false;
      }
    } else if (key == "--num-rings") {
      const char * v = need_value("--num-rings");
      if (!v || !parseInt(v, args.num_rings) || args.num_rings <= 0) {
        std::cerr << "Invalid --num-rings (must be > 0)" << std::endl;
        return false;
      }
    } else if (key == "--num-sectors") {
      const char * v = need_value("--num-sectors");
      if (!v || !parseInt(v, args.num_sectors) || args.num_sectors <= 0) {
        std::cerr << "Invalid --num-sectors (must be > 0)" << std::endl;
        return false;
      }
    } else if (key == "--height-min") {
      const char * v = need_value("--height-min");
      if (!v || !parseDouble(v, args.height_min)) {
        std::cerr << "Invalid --height-min" << std::endl;
        return false;
      }
    } else if (key == "--min-points") {
      const char * v = need_value("--min-points");
      if (!v || !parseInt(v, args.min_points) || args.min_points < 0) {
        std::cerr << "Invalid --min-points (must be >= 0)" << std::endl;
        return false;
      }
    } else {
      std::cerr << "Unknown argument: " << key << std::endl;
      return false;
    }
  }

  if (!args.show_help) {
    if (args.pcd_path.empty()) {
      std::cerr << "Missing required --pcd" << std::endl;
      return false;
    }
    if (args.output_path.empty()) {
      std::cerr << "Missing required --output" << std::endl;
      return false;
    }
  }
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  CliArgs args;
  if (!parseArgs(argc, argv, args)) {
    printUsage(std::cerr, argv[0]);
    return 1;
  }
  if (args.show_help) {
    printUsage(std::cout, argv[0]);
    return 0;
  }

  const auto t_start = std::chrono::steady_clock::now();

  // Load PCD.
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(args.pcd_path, *cloud) < 0) {
    std::cerr << "Failed to load PCD: " << args.pcd_path << std::endl;
    return 1;
  }
  if (cloud->empty()) {
    std::cerr << "PCD is empty: " << args.pcd_path << std::endl;
    return 1;
  }
  std::cout << "Loaded " << cloud->size() << " points from " << args.pcd_path << std::endl;

  // Compute AABB on x/y.
  double x_min = std::numeric_limits<double>::infinity();
  double x_max = -std::numeric_limits<double>::infinity();
  double y_min = std::numeric_limits<double>::infinity();
  double y_max = -std::numeric_limits<double>::infinity();
  for (const auto & pt : cloud->points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) {
      continue;
    }
    x_min = std::min(x_min, static_cast<double>(pt.x));
    x_max = std::max(x_max, static_cast<double>(pt.x));
    y_min = std::min(y_min, static_cast<double>(pt.y));
    y_max = std::max(y_max, static_cast<double>(pt.y));
  }
  if (!std::isfinite(x_min) || !std::isfinite(x_max) || !std::isfinite(y_min) ||
      !std::isfinite(y_max)) {
    std::cerr << "Failed to compute AABB (no finite x/y points)" << std::endl;
    return 1;
  }
  std::cout << "Map AABB: x=[" << x_min << ", " << x_max << "] y=[" << y_min << ", " << y_max
            << "]" << std::endl;

  // Build KdTree for radius search.
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);

  // Configure engine and DB.
  small_gicp_relocalization::ScanContextConfig cfg;
  cfg.num_rings = args.num_rings;
  cfg.num_sectors = args.num_sectors;
  cfg.max_radius = args.max_radius;
  cfg.height_min = args.height_min;
  small_gicp_relocalization::ScanContextEngine engine(cfg);
  small_gicp_relocalization::ScanContextDB db;
  db.config = cfg;

  // Enumerate sample poses on a regular grid.
  std::vector<Eigen::Vector2d> samples;
  for (double sy = y_min; sy <= y_max + 1e-6; sy += args.grid_spacing) {
    for (double sx = x_min; sx <= x_max + 1e-6; sx += args.grid_spacing) {
      samples.emplace_back(sx, sy);
    }
  }
  const size_t total = samples.size();
  std::cout << "Sampling " << total << " grid poses (spacing=" << args.grid_spacing
            << " m, max_radius=" << args.max_radius << " m)" << std::endl;

  size_t valid = 0;
  size_t skipped_low_density = 0;

  pcl::PointCloud<pcl::PointXYZ> local_cloud;
  std::vector<int> indices;
  std::vector<float> sq_dists;

  for (size_t k = 0; k < total; ++k) {
    const auto & s = samples[k];
    pcl::PointXYZ query;
    query.x = static_cast<float>(s.x());
    query.y = static_cast<float>(s.y());
    query.z = 0.0f;

    indices.clear();
    sq_dists.clear();
    const int n = kdtree.radiusSearch(
      query, static_cast<float>(args.max_radius), indices, sq_dists);
    if (n < args.min_points) {
      ++skipped_low_density;
    } else {
      local_cloud.points.clear();
      local_cloud.points.reserve(static_cast<size_t>(n));
      for (int idx : indices) {
        const auto & src = cloud->points[static_cast<size_t>(idx)];
        pcl::PointXYZ pt;
        pt.x = src.x - query.x;
        pt.y = src.y - query.y;
        pt.z = src.z;  // keep absolute z; height_min sentinel handles empty cells
        local_cloud.points.push_back(pt);
      }
      local_cloud.width = static_cast<uint32_t>(local_cloud.points.size());
      local_cloud.height = 1;
      local_cloud.is_dense = false;

      auto sc = engine.makeScanContext(local_cloud);
      auto rk = engine.makeRingKey(sc);

      db.poses.emplace_back(s.x(), s.y(), 0.0);
      db.descriptors.emplace_back(std::move(sc));
      db.ring_keys.emplace_back(std::move(rk));
      ++valid;
    }

    if ((k + 1) % 100 == 0 || (k + 1) == total) {
      std::cout << "processed " << (k + 1) << "/" << total << " (valid=" << valid
                << ", skipped_low_density=" << skipped_low_density << ")" << std::endl;
    }
  }

  if (db.poses.empty()) {
    std::cerr << "No valid samples produced (all below --min-points=" << args.min_points
              << "). Aborting." << std::endl;
    return 1;
  }

  if (!db.save(args.output_path)) {
    std::cerr << "Failed to save DB to " << args.output_path << std::endl;
    return 1;
  }

  std::uintmax_t file_size = 0;
  std::error_code ec;
  file_size = std::filesystem::file_size(args.output_path, ec);
  if (ec) {
    file_size = 0;
  }

  const auto t_end = std::chrono::steady_clock::now();
  const double elapsed_s =
    std::chrono::duration<double>(t_end - t_start).count();

  std::cout << "----- Summary -----" << std::endl;
  std::cout << "Total poses    : " << total << std::endl;
  std::cout << "Valid poses    : " << valid << std::endl;
  std::cout << "Skipped (sparse): " << skipped_low_density << std::endl;
  std::cout << "Output file    : " << args.output_path << std::endl;
  std::cout << "File size      : " << file_size << " bytes" << std::endl;
  std::cout << "Elapsed        : " << elapsed_s << " s" << std::endl;
  return 0;
}
