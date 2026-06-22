#!/usr/bin/env bash
# C 区(茂密森林)避障 —— 实飞启动脚本
#
# 一条命令搞定:建日志目录 → 启动 safe_landing → 同时把终端日志留底。
# 实飞用,默认【不开 RViz】(无线连接传点云会非常卡;RViz 留给离线复盘)。
#
# 用法:
#   ./run_c_zone.sh                # 默认 cruise_height=1.8
#   ./run_c_zone.sh 1.5            # 自定义巡航/限飞高度(米),务必 <2.0
#
# 飞完两份日志都在 ./logs/(随工作空间走,重启不丢):
#   flight_*.csv     节点自动写 —— 位姿/速度/最近障碍距离/限速系数/关键事件(查"撞没撞、卡在哪")
#   terminal_*.log   本脚本 tee  —— planner 聚类点数/膨胀值/无解原因(查"为什么")

set -euo pipefail

# 脚本所在目录 = 工作空间根,日志放它下面的 logs/,移动整个空间也不会错。
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$SCRIPT_DIR/logs"

CRUISE_HEIGHT="${1:-1.8}"
STAMP="$(date +%Y%m%d_%H%M%S)"

mkdir -p "$LOG_DIR"

echo "============================================================"
echo " C 区避障实飞"
echo "   cruise_height : ${CRUISE_HEIGHT} m  (限高 <2.0)"
echo "   RViz          : 关(实飞无线;离线复盘再开)"
echo "   CSV 日志      : ${LOG_DIR}/flight_*.csv      (节点自动写)"
echo "   终端日志      : ${LOG_DIR}/terminal_${STAMP}.log"
echo "============================================================"

# 注意:CSV 的写入目录由 config/safe_landing.yaml 的 log_dir 决定,
# 已设为本工作空间的 logs/,与此处终端日志同目录。
roslaunch safe_landing safe_landing.launch \
  cruise_height:="${CRUISE_HEIGHT}" \
  2>&1 | tee "${LOG_DIR}/terminal_${STAMP}.log"
