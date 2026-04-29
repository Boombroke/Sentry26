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

#ifndef SENTRY_MOTION_MANAGER__STUCK_DETECTOR_HPP_
#define SENTRY_MOTION_MANAGER__STUCK_DETECTOR_HPP_

#include <deque>

namespace sentry_motion_manager
{

struct StuckDetectorConfig
{
  double window_s{3.0};            // 滑动窗口长度
  double cmd_threshold_mps{0.05};  // 窗口内 |cmd_vx| 均值低于此视为无有效命令（不判卡）
  double position_threshold_m{0.10};  // 窗口内机器人最大位移低于此视为无进展
  double hold_s{1.0};                 // 连续满足卡住条件至少此秒数才触发
  double cooldown_s{3.0};             // 触发结束后至少冷却此秒数才允许再次触发
};

class StuckDetector
{
public:
  explicit StuckDetector(const StuckDetectorConfig & config = {});

  void configure(const StuckDetectorConfig & config);

  // 外部往窗口喂数据（时间戳单位秒；典型来源是 ROS 时间 now().seconds()）
  void addOdometry(double time_s, double x, double y);
  void addCommandLinearSpeed(double time_s, double abs_linear_x);

  // 每帧 tick 一次，返回本次 tick 后是否应向外发布 trigger=true
  // 内部自带 holdStart/cooldown 去抖。now_s >= 上一次 tick 的 now_s。
  bool tick(double now_s);

  // 外部通知"脱困已结束"（succeeded/failed/idle），用于允许下一次触发
  void notifyRecoveryFinished(double now_s);

  // 查询接口（用于 ROS 诊断发布）
  bool triggerActive() const;
  double lastWindowCmdMean() const;       // 最近一次 tick 计算的 cmd 均值
  double lastWindowDisplacement() const;  // 最近一次 tick 计算的窗口内最大位移
  double holdElapsed(double now_s) const;  // 当前已连续满足卡住条件的秒数（未满足则 <0）

private:
  struct OdomSample
  {
    double time_s{0.0};
    double x{0.0};
    double y{0.0};
  };

  struct CommandSample
  {
    double time_s{0.0};
    double abs_linear_x{0.0};
  };

  void pruneOdometry(double cutoff_s);
  void pruneCommands(double cutoff_s);
  double computeCommandMean() const;
  double computeMaxDisplacement() const;

  StuckDetectorConfig config_;

  std::deque<OdomSample> odom_samples_;
  std::deque<CommandSample> cmd_samples_;

  bool trigger_active_{false};
  bool hold_active_{false};
  double hold_start_s_{0.0};
  double last_trigger_end_s_{-1.0e12};

  double last_cmd_mean_{0.0};
  double last_displacement_{0.0};
};

}  // namespace sentry_motion_manager

#endif  // SENTRY_MOTION_MANAGER__STUCK_DETECTOR_HPP_
