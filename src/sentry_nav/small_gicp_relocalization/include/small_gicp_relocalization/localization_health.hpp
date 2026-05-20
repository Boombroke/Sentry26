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

#ifndef SMALL_GICP_RELOCALIZATION__LOCALIZATION_HEALTH_HPP_
#define SMALL_GICP_RELOCALIZATION__LOCALIZATION_HEALTH_HPP_

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <deque>
#include <mutex>
#include <vector>

namespace small_gicp_relocalization
{

struct HealthConfig
{
  size_t window_size = 5;                    // sliding window size
  double unhealthy_score_threshold = 50000;  // median below this -> unhealthy
  double stale_timeout_sec = 30.0;           // no fresh success for this long -> unhealthy
};

struct HealthMetrics
{
  size_t sample_count = 0;
  double median_score = 0.0;
  double min_score = 0.0;
  double max_score = 0.0;
  double seconds_since_last_success = 0.0;
  bool unhealthy = false;
};

// Sliding-window health monitor for the GICP relocalization stack.
//
// Fed by all three layers (periodic / emergency / deep): the score definition
// matches the emergency path, score = num_inliers / (fitness_error + 0.001).
// The class is thread-safe so it can be called from both the main spinner and
// the deep-verification worker thread.
class LocalizationHealth
{
public:
  explicit LocalizationHealth(const HealthConfig & cfg = {})
  : cfg_(cfg), last_success_time_(std::chrono::steady_clock::now())
  {
  }

  // Record a successful GICP result.
  void recordSuccess(double score)
  {
    std::lock_guard<std::mutex> lock(mu_);
    recent_scores_.push_back(score);
    while (recent_scores_.size() > cfg_.window_size) {
      recent_scores_.pop_front();
    }
    last_success_time_ = std::chrono::steady_clock::now();
    has_any_success_ = true;
    ++success_count_;
  }

  // Record a failed GICP attempt. Does not enter the score window; only used
  // for diagnostics counters.
  void recordFailure()
  {
    std::lock_guard<std::mutex> lock(mu_);
    ++failure_count_;
  }

  bool isUnhealthy() const
  {
    std::lock_guard<std::mutex> lock(mu_);
    return computeUnhealthyLocked();
  }

  HealthMetrics getMetrics() const
  {
    std::lock_guard<std::mutex> lock(mu_);
    HealthMetrics m;
    m.sample_count = recent_scores_.size();
    if (!recent_scores_.empty()) {
      std::vector<double> sorted(recent_scores_.begin(), recent_scores_.end());
      std::sort(sorted.begin(), sorted.end());
      const size_t n = sorted.size();
      if (n % 2 == 1) {
        m.median_score = sorted[n / 2];
      } else {
        m.median_score = 0.5 * (sorted[n / 2 - 1] + sorted[n / 2]);
      }
      m.min_score = sorted.front();
      m.max_score = sorted.back();
    }
    m.seconds_since_last_success = secondsSinceLastSuccessLocked();
    m.unhealthy = computeUnhealthyLocked();
    return m;
  }

  void reset()
  {
    std::lock_guard<std::mutex> lock(mu_);
    recent_scores_.clear();
    has_any_success_ = false;
    success_count_ = 0;
    failure_count_ = 0;
    last_success_time_ = std::chrono::steady_clock::now();
  }

  const HealthConfig & config() const { return cfg_; }

private:
  double secondsSinceLastSuccessLocked() const
  {
    if (!has_any_success_) {
      return 0.0;
    }
    const auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(now - last_success_time_).count();
  }

  bool computeUnhealthyLocked() const
  {
    // Stale-timeout check applies as soon as we have at least one success.
    if (has_any_success_) {
      const double idle = secondsSinceLastSuccessLocked();
      if (idle > cfg_.stale_timeout_sec) {
        return true;
      }
    }

    // Insufficient samples -> withhold judgement.
    if (recent_scores_.size() < cfg_.window_size) {
      return false;
    }

    std::vector<double> sorted(recent_scores_.begin(), recent_scores_.end());
    std::sort(sorted.begin(), sorted.end());
    const size_t n = sorted.size();
    double median = (n % 2 == 1)
      ? sorted[n / 2]
      : 0.5 * (sorted[n / 2 - 1] + sorted[n / 2]);
    return median < cfg_.unhealthy_score_threshold;
  }

  HealthConfig cfg_;
  mutable std::mutex mu_;
  std::deque<double> recent_scores_;
  std::chrono::steady_clock::time_point last_success_time_;
  bool has_any_success_ = false;
  size_t failure_count_ = 0;
  size_t success_count_ = 0;
};

}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__LOCALIZATION_HEALTH_HPP_
