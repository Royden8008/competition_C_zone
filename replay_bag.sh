#!/usr/bin/env bash
# C 区 —— 离线回放脚本(一条命令把"放包 + 局部地图 + RViz"全起来)
#
# 录的包是"数据录像";本脚本把它重新播一遍,系统其它程序就以为雷达又活了,
# 于是能离线复盘:定位漂没漂、点云有没有树、新节点 /cloud_local_map 是否更密。
#
# 用法:
#   ./replay_bag.sh                       # 放 logs/ 里【最新】的包(最常用)
#   ./replay_bag.sh c_zone_2026....bag    # 指定 logs/ 里的某个包(可省略 .bag)
#   ./replay_bag.sh /绝对/路径/xxx.bag    # 指定任意路径的包
#
# 播放中:空格=暂停/继续,Ctrl-C=停止(会自动把 RViz / 局部地图节点一起关掉)。

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$SCRIPT_DIR/logs"

# --- source 五件套(回放也要,跟实飞一样) ---
source /opt/ros/noetic/setup.bash
source /home/cwkj/ws_livox/devel/setup.bash
source /home/cwkj/fast_lio2_ws/devel/setup.bash
source /home/cwkj/cwkj_ws/devel/setup.bash
source "$SCRIPT_DIR/devel/setup.bash"

# --- 选包:给了参数就用参数,没给就挑 logs/ 里最新的 ---
if [ "$#" -ge 1 ]; then
  ARG="$1"
  if [ -f "$ARG" ]; then            BAG="$ARG"               # 直接给了能找到的路径
  elif [ -f "$LOG_DIR/$ARG" ]; then BAG="$LOG_DIR/$ARG"      # logs/ 里的文件名
  elif [ -f "$LOG_DIR/$ARG.bag" ]; then BAG="$LOG_DIR/$ARG.bag"  # 省了 .bag 后缀
  else
    echo "✗ 找不到这个包:$ARG"
    echo "  logs/ 里现有的包:"; ls -1t "$LOG_DIR"/*.bag 2>/dev/null | sed 's#.*/#   #' || echo "   (空)"
    exit 1
  fi
else
  BAG="$(ls -1t "$LOG_DIR"/*.bag 2>/dev/null | head -1)"
  if [ -z "$BAG" ]; then
    echo "✗ logs/ 里没有任何 .bag,先用 ./record_bag.sh 录一个。"; exit 1
  fi
fi

# --- 没有 roscore 就起一个(并记下来,退出时关掉) ---
STARTED_ROSCORE=0
if ! rostopic list >/dev/null 2>&1; then
  echo "· 没检测到 roscore,自动起一个 ..."
  roscore >/dev/null 2>&1 &
  STARTED_ROSCORE=1
  until rostopic list >/dev/null 2>&1; do sleep 0.3; done
fi

# 用包里的时间,不用墙上时间(局部地图按 1.5s 淘汰旧帧,必须靠它才算得对)
rosparam set use_sim_time true

# --- 退出时统一收尾:把本脚本起的后台进程都关掉 ---
PIDS=()
cleanup() {
  echo; echo "· 收尾,关闭 RViz / 局部地图节点 ..."
  for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null; done
  [ "$STARTED_ROSCORE" -eq 1 ] && { echo "· 关闭自动起的 roscore ..."; killall -q roscore rosmaster rosout 2>/dev/null; }
}
trap cleanup EXIT INT TERM

# --- 后台起:滚动局部地图节点(现算 /cloud_local_map,包里没录这个话题) ---
rosrun safe_landing local_obstacle_map_node \
  _input_topic:=/cloud_registered _output_topic:=/cloud_local_map \
  _odom_topic:=/Odometry _max_age:=1.5 _max_range:=6.0 \
  _voxel_leaf:=0.10 _max_frames:=40 >/dev/null 2>&1 &
PIDS+=("$!")

# --- 后台起:RViz(Fixed Frame 已设为 camera_init) ---
rviz -d "$SCRIPT_DIR/src/safe_landing/rviz/safe_landing.rviz" >/dev/null 2>&1 &
PIDS+=("$!")

echo "============================================================"
echo " 离线回放"
echo "   包       : $BAG"
echo "   局部地图 : 已起(/cloud_registered → /cloud_local_map)"
echo "   RViz     : 已起(Fixed Frame=camera_init)"
echo "   控制     : 空格=暂停/继续   Ctrl-C=停止并收尾"
echo "   注意     : 已开 --loop 循环播放(包只有几十秒,循环着才方便在 RViz 里设置)"
echo "============================================================"

# --- 前台放包:--clock 配合 use_sim_time;--loop 循环,免得放完话题就没了 ---
rosbag play --clock --loop "$BAG"
