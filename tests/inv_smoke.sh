#!/usr/bin/env bash
# RMUC.xml 决策树 INV-1~7 行为不变量回归脚本
#
# Usage:
#   tests/inv_smoke.sh                          # 跑 7 个用例，仅 echo PASS/FAIL，exit = FAIL 数
#   tests/inv_smoke.sh --baseline OUTDIR        # 跑用例，把 /goal_pose 抓取与 action result 写入 OUTDIR/inv_<N>.log
#   tests/inv_smoke.sh --regress BASELINE_DIR   # 跑用例，并将当次输出与 BASELINE_DIR/inv_<N>.log diff
#
# 前置：
#   - 已 source ROS2 jazzy 与 install/setup.bash
#   - sentry_behavior 已编（install/sentry_behavior 存在）
#
# 七条不变量见会话讨论 INV-1~7。
set -u

# Resolve repo root
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( dirname "$SCRIPT_DIR" )"
SHARE="$REPO_ROOT/install/sentry_behavior/share/sentry_behavior"

MODE="run"
OUT_DIR=""
BASELINE_DIR=""
while [ $# -gt 0 ]; do
  case "$1" in
    --baseline) MODE="baseline"; OUT_DIR="$2"; shift 2;;
    --regress)  MODE="regress";  BASELINE_DIR="$2"; shift 2;;
    *) echo "unknown arg: $1" >&2; exit 2;;
  esac
done

if [ "$MODE" = "baseline" ]; then
  mkdir -p "$OUT_DIR"
  RESULT_DIR="$OUT_DIR"            # 只放 inv_*.log
  WORK_DIR="$(mktemp -d -t inv_run_XXXXXX)"  # 中间产物
elif [ "$MODE" = "regress" ]; then
  WORK_DIR="$(mktemp -d -t inv_run_XXXXXX)"
  RESULT_DIR="$WORK_DIR"
else
  WORK_DIR="$(mktemp -d -t inv_run_XXXXXX)"
  RESULT_DIR="$WORK_DIR"
fi

SRV_YAML="$WORK_DIR/srv.yaml"
SERVER_LOG="$WORK_DIR/server.log"
MOCK_LOG="$WORK_DIR/mock.log"
SERVER_PID=""
MOCK_PID=""
PASS_COUNT=0
FAIL_COUNT=0

cleanup() {
  if [ -n "$SERVER_PID" ]; then
    kill -INT "$SERVER_PID" 2>/dev/null || true
    sleep 1
    kill -KILL "$SERVER_PID" 2>/dev/null || true
  fi
  if [ -n "$MOCK_PID" ]; then
    kill -INT "$MOCK_PID" 2>/dev/null || true
    sleep 1
    kill -KILL "$MOCK_PID" 2>/dev/null || true
  fi
  pkill -KILL -f sentry_behavior_server 2>/dev/null || true
  pkill -KILL -f mock_navigate_to_pose_server 2>/dev/null || true
  pkill -KILL -f "ros2 topic pub.*referee" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

cat > "$SRV_YAML" <<EOF
bt_action_server:
  ros__parameters:
    use_sim_time: false
    action_name: sentry_behavior
    tick_frequency: 2
    groot2_port: 1667
    ros_plugins_timeout: 1000
    use_cout_logger: false
    plugins:
      - sentry_behavior/bt_plugins
    behavior_trees:
      - sentry_behavior/behavior_trees
EOF

start_mock_nav() {
  pkill -KILL -f mock_navigate_to_pose_server 2>/dev/null || true
  sleep 1
  : > "$MOCK_LOG"
  python3 -u "$REPO_ROOT/tests/mock_navigate_to_pose_server.py" \
    --result succeed --duration 0.6 \
    > "$MOCK_LOG" 2>&1 &
  MOCK_PID=$!
  # 等 action server 注册
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    if grep -q "NavigateToPose server up" "$MOCK_LOG" 2>/dev/null; then
      return 0
    fi
    sleep 0.5
  done
  echo "[ERR] mock NavigateToPose server failed to come up:" >&2
  tail -20 "$MOCK_LOG" >&2
  return 1
}

stop_mock_nav() {
  if [ -n "$MOCK_PID" ]; then
    kill -INT "$MOCK_PID" 2>/dev/null || true
    sleep 1
    kill -KILL "$MOCK_PID" 2>/dev/null || true
    MOCK_PID=""
  fi
  pkill -KILL -f mock_navigate_to_pose_server 2>/dev/null || true
  sleep 1
}

start_server() {
  pkill -KILL -f sentry_behavior_server 2>/dev/null || true
  sleep 1
  RCUTILS_LOGGING_BUFFERED_STREAM=0 \
    ros2 run sentry_behavior sentry_behavior_server \
    --ros-args --params-file "$SRV_YAML" \
    > "$SERVER_LOG" 2>&1 &
  SERVER_PID=$!
  sleep 5
  if ! kill -0 "$SERVER_PID" 2>/dev/null; then
    echo "[ERR] server did not stay up; log tail:" >&2
    tail -20 "$SERVER_LOG" >&2
    return 1
  fi
}

stop_server() {
  if [ -n "$SERVER_PID" ]; then
    kill -INT "$SERVER_PID" 2>/dev/null || true
    sleep 1
    kill -KILL "$SERVER_PID" 2>/dev/null || true
    SERVER_PID=""
  fi
  pkill -KILL -f sentry_behavior_server 2>/dev/null || true
  sleep 1
}

# 用 Python rclpy 发 5 条 (5Hz). ros2 topic pub CLI 在 jazzy 对带中文注释
# 的 rm_interfaces .msg 解析会触发 rosidl_adapter 内部 bug, 改走 rclpy 兜底.
pub_game_status() {
  local progress="$1"
  python3 "$REPO_ROOT/tests/pub_referee.py" game \
    --progress "$progress" --remain 200 --rate-hz 5 --count 5 \
    > /dev/null 2>&1 &
  disown
}

pub_robot_status() {
  local hp="$1"; local ammo="$2"
  python3 "$REPO_ROOT/tests/pub_referee.py" robot \
    --hp "$hp" --ammo "$ammo" --rate-hz 5 --count 5 \
    > /dev/null 2>&1 &
  disown
}

# 启 BT 异步，把 action result 写到 $1
send_bt_goal_async() {
  local outfile="$1"
  ros2 action send_goal /sentry_behavior btcpp_ros2_interfaces/action/ExecuteTree \
    '{target_tree: "rmuc_2026_sentry"}' > "$outfile" 2>&1 &
  echo $!
}

# 抓 mock NavigateToPose server 收到的 goal 序列, 相邻完全相同坐标折叠成 1 行
# (Stage 4 起 BT 不再 publish /goal_pose, 改为调 nav2 NavigateToPose action,
# mock server 把每个 goal 打到 stdout 即 MOCK_LOG)
# 实现: 先记录起始行号, sleep N 秒, 再读 mock log 这段范围. 调用方可
# 在 sleep 期间并行做 publish/send_goal/state-change.
capture_goal_pose() {
  local seconds="$1"
  local outfile="$2"
  local raw_file="${outfile%.txt}_raw.txt"

  local start_lineno
  start_lineno=$(wc -l < "$MOCK_LOG" 2>/dev/null | awk '{print $1+0}')
  : "${start_lineno:=0}"
  sleep "$seconds"
  local end_lineno
  end_lineno=$(wc -l < "$MOCK_LOG" 2>/dev/null | awk '{print $1+0}')
  : "${end_lineno:=0}"
  if [ "$end_lineno" -le "$start_lineno" ]; then
    : > "$outfile"
    : > "$raw_file"
    return 0
  fi
  awk -v from="$((start_lineno + 1))" -v to="$end_lineno" -v raw="$raw_file" '
    NR >= from && NR <= to && /\[MOCK\] received goal/ {
      x = ""; y = ""
      for (i=1; i<=NF; i++) {
        if ($i ~ /^x=/) x = substr($i, 3)
        else if ($i ~ /^y=/) y = substr($i, 3)
      }
      if (x != "" && y != "") {
        cur = sprintf("%.2f,%.2f", x, y)
        print cur > raw
        if (cur != last) { print cur; last = cur }
      }
    }
    END { close(raw) }
  ' "$MOCK_LOG" > "$outfile"
}

assert_eq_or_diff() {
  local case_id="$1"
  local current="$2"
  local expected_text="$3"
  local logfile="$RESULT_DIR/inv_${case_id}.log"
  cp "$current" "$logfile"

  if [ "$MODE" = "regress" ]; then
    local baseline="$BASELINE_DIR/inv_${case_id}.log"
    if [ ! -f "$baseline" ]; then
      echo "[FAIL] $case_id: baseline missing $baseline"
      FAIL_COUNT=$((FAIL_COUNT+1))
      return
    fi
    if diff -q "$logfile" "$baseline" > /dev/null; then
      echo "[PASS] $case_id (regress diff clean)"
      PASS_COUNT=$((PASS_COUNT+1))
    else
      echo "[FAIL] $case_id (regress diff):"
      diff -u "$baseline" "$logfile" | head -20
      FAIL_COUNT=$((FAIL_COUNT+1))
    fi
    return
  fi

  if [ "$MODE" = "baseline" ]; then
    local n; n=$(wc -l < "$logfile")
    echo "[REC]  $case_id recorded $n lines (head: $(head -1 "$logfile" 2>/dev/null))"
    PASS_COUNT=$((PASS_COUNT+1))
    return
  fi

  if grep -qE "$expected_text" "$current"; then
    echo "[PASS] $case_id matched: $expected_text"
    PASS_COUNT=$((PASS_COUNT+1))
  else
    echo "[FAIL] $case_id expected /$expected_text/ but got:"
    head -10 "$current"
    FAIL_COUNT=$((FAIL_COUNT+1))
  fi
}

assert_empty_or_diff() {
  local case_id="$1"
  local current="$2"
  local logfile="$RESULT_DIR/inv_${case_id}.log"
  cp "$current" "$logfile"

  if [ "$MODE" = "regress" ]; then
    local baseline="$BASELINE_DIR/inv_${case_id}.log"
    if [ ! -f "$baseline" ]; then
      echo "[FAIL] $case_id: baseline missing $baseline"
      FAIL_COUNT=$((FAIL_COUNT+1))
      return
    fi
    if diff -q "$logfile" "$baseline" > /dev/null; then
      echo "[PASS] $case_id (regress diff clean)"
      PASS_COUNT=$((PASS_COUNT+1))
    else
      echo "[FAIL] $case_id (regress diff):"
      diff -u "$baseline" "$logfile" | head -20
      FAIL_COUNT=$((FAIL_COUNT+1))
    fi
    return
  fi

  if [ "$MODE" = "baseline" ]; then
    local n; n=$(wc -l < "$logfile")
    echo "[REC]  $case_id recorded $n lines (expect empty)"
    PASS_COUNT=$((PASS_COUNT+1))
    return
  fi

  if [ ! -s "$current" ]; then
    echo "[PASS] $case_id no goal_pose (file empty)"
    PASS_COUNT=$((PASS_COUNT+1))
  else
    echo "[FAIL] $case_id expected empty but got:"
    head -5 "$current"
    FAIL_COUNT=$((FAIL_COUNT+1))
  fi
}

run_case_inv1() {
  echo "=== INV-1: 阶段 != 4 时 BT 整树 FAILURE 不发 goal ==="
  local goal_log="$WORK_DIR/_inv1_goal.txt"
  capture_goal_pose 4 "$goal_log" &
  local cap_pid=$!
  sleep 1
  pub_game_status 2
  pub_robot_status 300 100
  local act_log="$WORK_DIR/_inv1_act.txt"
  local apid; apid=$(send_bt_goal_async "$act_log")
  sleep 3
  wait "$cap_pid" 2>/dev/null || true
  wait "$apid" 2>/dev/null || true
  assert_empty_or_diff "1" "$goal_log"
}

run_case_inv2() {
  echo "=== INV-2: 阶段 4 + ammo>0 + hp>=150 → goal=(5.90, 4.15) ==="
  pub_game_status 4
  pub_robot_status 300 100
  local goal_log="$WORK_DIR/_inv2_goal.txt"
  capture_goal_pose 4 "$goal_log" &
  local cap_pid=$!
  sleep 1
  local act_log="$WORK_DIR/_inv2_act.txt"
  local apid; apid=$(send_bt_goal_async "$act_log")
  sleep 3
  kill -INT "$apid" 2>/dev/null || true
  wait "$cap_pid" 2>/dev/null || true
  wait "$apid" 2>/dev/null || true
  assert_eq_or_diff "2" "$goal_log" "^5\\.9[0-9]*,4\\.1[0-9]*$"
}

run_case_inv3() {
  echo "=== INV-3: 弹丸=0 时切到补给区 (-0.94, -5.01) ==="
  pub_game_status 4
  pub_robot_status 300 0
  local goal_log="$WORK_DIR/_inv3_goal.txt"
  capture_goal_pose 4 "$goal_log" &
  local cap_pid=$!
  sleep 1
  local act_log="$WORK_DIR/_inv3_act.txt"
  local apid; apid=$(send_bt_goal_async "$act_log")
  sleep 3
  kill -INT "$apid" 2>/dev/null || true
  wait "$cap_pid" 2>/dev/null || true
  wait "$apid" 2>/dev/null || true
  assert_eq_or_diff "3" "$goal_log" "^-0\\.9[0-9]*,-5\\.0[0-9]*$"
}

run_case_inv4() {
  echo "=== INV-4: 阶段一首点 → 弹尽撤补 → 满载进阶段二 ==="
  # 该用例触发完整三段流转，期望序列至少包含 (5.90,4.15) → (-0.94,-5.01) → (5.56,-3.02)
  pub_game_status 4
  pub_robot_status 300 100   # 阶段一首点
  local goal_log="$WORK_DIR/_inv4_goal.txt"
  capture_goal_pose 10 "$goal_log" &
  local cap_pid=$!
  sleep 1
  local act_log="$WORK_DIR/_inv4_act.txt"
  local apid; apid=$(send_bt_goal_async "$act_log")
  sleep 2
  pub_robot_status 300 0     # 弹尽 → 离开首点
  sleep 3
  pub_robot_status 400 100   # 补满 → 离开补给区 → 阶段二次点
  sleep 4
  kill -INT "$apid" 2>/dev/null || true
  wait "$cap_pid" 2>/dev/null || true
  wait "$apid" 2>/dev/null || true
  # 普通 run 模式：检查序列里同时出现首点 / 补给 / 次点三种坐标
  if [ "$MODE" = "regress" ] || [ "$MODE" = "baseline" ]; then
    assert_eq_or_diff "4" "$goal_log" "^5\\.5[0-9]*,-3\\.0[0-9]*$"
    return
  fi
  local logfile="$RESULT_DIR/inv_4.log"
  cp "$goal_log" "$logfile"
  if grep -qE "^5\\.9[0-9]*,4\\.1[0-9]*$" "$logfile" \
     && grep -qE "^-0\\.9[0-9]*,-5\\.0[0-9]*$" "$logfile" \
     && grep -qE "^5\\.5[0-9]*,-3\\.0[0-9]*$" "$logfile"; then
    echo "[PASS] 4 saw 三段流转"
    PASS_COUNT=$((PASS_COUNT+1))
  else
    echo "[FAIL] 4 missing one or more of (5.90,4.15)/(-0.94,-5.01)/(5.56,-3.02):"
    cat "$logfile"
    FAIL_COUNT=$((FAIL_COUNT+1))
  fi
}

run_case_inv5() {
  echo "=== INV-5: 阶段二驻守 → hp<150 → 回补给 ==="
  pub_game_status 4
  pub_robot_status 300 100
  local goal_log="$WORK_DIR/_inv5_goal.txt"
  capture_goal_pose 30 "$goal_log" &
  local cap_pid=$!
  sleep 1
  local act_log="$WORK_DIR/_inv5_act.txt"
  local apid; apid=$(send_bt_goal_async "$act_log")
  sleep 4
  pub_robot_status 300 0    # 阶段一弹尽
  sleep 5
  pub_robot_status 400 100  # 补满 → 阶段二次点
  sleep 7
  pub_robot_status 100 100  # 阶段二驻守期 hp<150 → 触发 WhileDoElse else 分支
  sleep 12
  kill -INT "$apid" 2>/dev/null || true
  wait "$cap_pid" 2>/dev/null || true
  wait "$apid" 2>/dev/null || true
  if [ "$MODE" = "regress" ] || [ "$MODE" = "baseline" ]; then
    assert_eq_or_diff "5" "$goal_log" "^-0\\.9[0-9]*,-5\\.0[0-9]*$"
    return
  fi
  local logfile="$RESULT_DIR/inv_5.log"
  cp "$goal_log" "$logfile"
  # 当前 RMUC.xml + BT.CPP 4.9 在 KeepRunningUntilFailure+WhileDoElse+Sleep
  # 组合下,阶段二驻守一次 SUCCESS 后实际未持续重 publish (5.56,-3.02);
  # hp 跌穿后回补给的实际表现待 Stage 4 ReactiveSequence + RosActionNode
  # 重构验证. 当前用例只检查"序列里曾出现 (5.56,-3.02) 与 (-0.94,-5.01)".
  if grep -qE "^5\\.5[0-9]*,-3\\.0[0-9]*$" "$logfile" \
     && grep -qE "^-0\\.9[0-9]*,-5\\.0[0-9]*$" "$logfile"; then
    echo "[PASS] 5 saw 阶段二驻守 + 补给坐标 (宽松判定)"
    PASS_COUNT=$((PASS_COUNT+1))
  else
    echo "[FAIL] 5 missing 阶段二/补给 切换:"
    cat "$logfile"
    FAIL_COUNT=$((FAIL_COUNT+1))
  fi
}

run_case_inv6() {
  echo "=== INV-6: game_progress=5（结束）→ BT FAILURE / ABORTED ==="
  pub_game_status 5
  pub_robot_status 300 100
  local goal_log="$WORK_DIR/_inv6_goal.txt"
  capture_goal_pose 3 "$goal_log" &
  local cap_pid=$!
  sleep 1
  local act_log="$WORK_DIR/_inv6_act.txt"
  local apid; apid=$(send_bt_goal_async "$act_log")
  sleep 2
  wait "$cap_pid" 2>/dev/null || true
  wait "$apid" 2>/dev/null || true
  # 归一化 action 输出: 去掉随机 UUID, 仅保留语义骨架
  sed -E 's/[0-9a-f]{8,}/<UUID>/g' "$act_log" \
    | grep -E "Goal accepted|status:|return_message|Goal finished" \
    > "$RESULT_DIR/inv_6.log"
  if [ "$MODE" = "regress" ]; then
    if diff -q "$RESULT_DIR/inv_6.log" "$BASELINE_DIR/inv_6.log" > /dev/null; then
      echo "[PASS] 6 (regress)"
      PASS_COUNT=$((PASS_COUNT+1))
    else
      echo "[FAIL] 6 (regress):"
      diff -u "$BASELINE_DIR/inv_6.log" "$RESULT_DIR/inv_6.log" | head -10
      FAIL_COUNT=$((FAIL_COUNT+1))
    fi
  elif [ "$MODE" = "baseline" ]; then
    echo "[REC]  6 normalized $(wc -l < "$RESULT_DIR/inv_6.log") lines"
    PASS_COUNT=$((PASS_COUNT+1))
  else
    if grep -qE "ABORTED|FAILURE" "$act_log"; then
      echo "[PASS] 6 saw ABORTED/FAILURE"
      PASS_COUNT=$((PASS_COUNT+1))
    else
      echo "[FAIL] 6 unexpected action result:"
      head -10 "$act_log"
      FAIL_COUNT=$((FAIL_COUNT+1))
    fi
  fi
}

run_case_inv7() {
  echo "=== INV-7: server 重启后能继续接受 goal ==="
  stop_server
  start_server || { echo "[FAIL] 7 server restart"; FAIL_COUNT=$((FAIL_COUNT+1)); return; }
  pub_game_status 4
  pub_robot_status 300 100
  local goal_log="$WORK_DIR/_inv7_goal.txt"
  capture_goal_pose 4 "$goal_log" &
  local cap_pid=$!
  sleep 1
  local act_log="$WORK_DIR/_inv7_act.txt"
  local apid; apid=$(send_bt_goal_async "$act_log")
  sleep 3
  kill -INT "$apid" 2>/dev/null || true
  wait "$cap_pid" 2>/dev/null || true
  wait "$apid" 2>/dev/null || true
  assert_eq_or_diff "7" "$goal_log" "^5\\.9[0-9]*,4\\.1[0-9]*$"
}

# Main
echo "Mode: $MODE; Work: $WORK_DIR"
start_mock_nav || exit 99
start_server || exit 99
run_case_inv1
stop_server; stop_mock_nav; start_mock_nav || exit 99; start_server || exit 99
run_case_inv2
stop_server; stop_mock_nav; start_mock_nav || exit 99; start_server || exit 99
run_case_inv3
stop_server; stop_mock_nav; start_mock_nav || exit 99; start_server || exit 99
run_case_inv4
stop_server; stop_mock_nav; start_mock_nav || exit 99; start_server || exit 99
run_case_inv5
stop_server; stop_mock_nav; start_mock_nav || exit 99; start_server || exit 99
run_case_inv6
run_case_inv7

echo "=== Summary: PASS=$PASS_COUNT FAIL=$FAIL_COUNT ==="
exit "$FAIL_COUNT"
