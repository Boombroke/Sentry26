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

#ifndef SENTRY_MOTION_MANAGER__MOTION_TYPES_HPP_
#define SENTRY_MOTION_MANAGER__MOTION_TYPES_HPP_

#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"

namespace sentry_motion_manager
{

enum class MotionSource {
  kNavigation,
  kRecovery,
  kTerrain,
  kEvasion,
  kManual,
};

enum class MotionMode {
  kIdle,
  kNavigation,
  kRecovery,
  kTerrainTraverse,
  kEvasiveSpin,
  kManual,
  kEmergencyStop,
};

enum class RecoveryPhase {
  kInactive,
  kStraightReverse,
  kArcEscape,
  kFailed,
  kSucceeded,
};

struct MotionCommand
{
  MotionSource source{MotionSource::kNavigation};
  geometry_msgs::msg::TwistStamped twist{};
  bool valid{false};
};

struct MotionState
{
  MotionMode mode{MotionMode::kIdle};
  MotionSource selected_source{MotionSource::kNavigation};
  RecoveryPhase recovery_phase{RecoveryPhase::kInactive};
  bool output_enabled{false};
  bool emergency_stop{false};
  bool has_fresh_command{false};
};

std::string toString(MotionSource source);
std::string toString(MotionMode mode);
std::string toString(RecoveryPhase phase);

}  // namespace sentry_motion_manager

#endif  // SENTRY_MOTION_MANAGER__MOTION_TYPES_HPP_
