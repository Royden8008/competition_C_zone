#!/usr/bin/env bash
# C 区(茂密森林)录包脚本 —— 手动按需运行,跟飞行脚本分开。
#
# 单独开一个终端跑这个;不是每次飞都要录(包很大)。怀疑定位漂 / 点云漏树,
# 或想离线复盘时才录。录完按 Ctrl-C 停。
#
# 用法:
#   ./record_bag.sh              # 录默认话题到 logs/c_zone_<时间戳>.bag
#
# 包和 CSV、终端日志躺在同一个 logs/ 目录,时间戳能对上。
#
# ⚠️ 话题名各机器可能不同(雷达常见 /livox/lidar,也可能别的)。
#    脚本会先用 rostopic list 检查,缺哪个会提示你——按提示改下面 TOPICS 即可。

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$SCRIPT_DIR/logs"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG="${LOG_DIR}/c_zone_${STAMP}.bag"

# 要录的话题。名字对不上就改这里(用 rostopic list 看真名)。
TOPICS=(
  /livox/lidar          # 原始雷达 —— 可离线换 FAST-LIO 参数重跑(查定位漂)
  /livox/imu            # 原始 IMU  —— 同上
  /Odometry             # FAST-LIO 里程计
  /cloud_registered     # 世界系点云 —— 规划/避障/降落区评估直接吃这个
  /mavros/local_position/odom
  /mavros/vision_pose/pose   # 喂给飞控的位姿
  /mavros/state
  /mavros/setpoint_raw/local # 自主时的控制指令
  /tf
  /tf_static
)

# --- 开录前检查:roscore 在不在、话题缺不缺 ---
if ! rostopic list >/dev/null 2>&1; then
  echo "✗ 连不上 roscore —— 先把硬件/定位(终端1)和避障(终端2)起起来再录。"
  exit 1
fi

LIVE="$(rostopic list 2>/dev/null)"
MISSING=()
for t in "${TOPICS[@]}"; do
  grep -qx "$t" <<<"$LIVE" || MISSING+=("$t")
done
if [ "${#MISSING[@]}" -gt 0 ]; then
  echo "⚠️ 以下话题当前不存在(会录成空):"
  printf '   %s\n' "${MISSING[@]}"
  echo "   → 用 'rostopic list' 看真名,改本脚本里的 TOPICS;确认无误可回车继续,Ctrl-C 取消。"
  read -r _
fi

mkdir -p "$LOG_DIR"

echo "============================================================"
echo " 录包中 → ${BAG}"
echo " 话题数 : ${#TOPICS[@]}   压缩 : lz4"
echo " 停止录制 : Ctrl-C"
echo "============================================================"

# --split --size=1024 可按需加(每卷 1GB 分卷,防单包过大)
exec rosbag record -O "$BAG" --lz4 "${TOPICS[@]}"
