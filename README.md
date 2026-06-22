# competition_C_ws

C 区(茂密森林)任务的**独立开发工作空间**。

从 `competition_ws_school` 拆出来,只包含 C 区需要的 `safe_landing`(纯避障),
在这里改代码、调参不会污染原工作空间。

## 任务说明

无人机自主穿过乱序排列的仿真树(高约 2m、直径约 0.5m),**飞行高度不得超过 2m**
(超过该项 0 分),稳定飞过判分线得 100 分。外部绕行不得分。树木位置赛前随机变更——
**不能背航点,必须靠实时定位 + 点云 + 规划/反应避障**。

C 区是"纯避障"任务,不需要视觉穿越/目标识别,因此本空间只带 `safe_landing`,
不含 `ring_gate_controller` / `ring_gate_perception`。

## 目录结构

```text
competition_C_ws/
├── README.md
├── docs/
│   └── C_zone_debug.md        # C 区调试指南(录包/离线复盘/分环节排查/常见坑)
└── src/
    ├── CMakeLists.txt         # catkin 顶层(符号链接)
    └── safe_landing/          # 避障 + 降落(C++),从原空间拷贝的独立副本
        ├── src/               # waypoint_follower / grid_planner /
        │                      #   landing_zone_evaluator / guarded_descent /
        │                      #   precision_landing / safe_landing_node
        ├── include/safe_landing/
        ├── config/safe_landing.yaml
        ├── launch/            # safe_landing.launch 等
        └── rviz/safe_landing.rviz
```

## 依赖

`safe_landing` 只依赖标准 ROS Noetic + `mavros_msgs` + PCL/Eigen,
`ar_track_alvar_msgs` 为可选(缺失时 marker 模式自动禁用,不影响编译/C 区)。

运行时依赖以下外部工作空间(不在本空间内,需各自先 source):

- `ws_livox`        — Livox Mid360 驱动
- `fast_lio2_ws`    — FAST-LIO,发布 `/cloud_registered` 与 `/Odometry`
- `cwkj_ws`         — `robot_bringup`(PX4/MAVROS、相机等)

## 编译

```bash
source /opt/ros/noetic/setup.bash
source /home/cwkj/ws_livox/devel/setup.bash
source /home/cwkj/fast_lio2_ws/devel/setup.bash
source /home/cwkj/cwkj_ws/devel/setup.bash

cd /home/cwkj/competition_C_ws
catkin_make
```

## 环境 source(每个新终端)

```bash
source /opt/ros/noetic/setup.bash
source /home/cwkj/ws_livox/devel/setup.bash
source /home/cwkj/fast_lio2_ws/devel/setup.bash
source /home/cwkj/cwkj_ws/devel/setup.bash
source /home/cwkj/competition_C_ws/devel/setup.bash
```

## 独立测 C 区避障(现有 launch,无需改代码)

`safe_landing.launch` 默认就是独立模式(自带 起飞 → 巡航 → 评估 → 下降 → 降落),
`config` / `cruise_height` 等都是启动参数:

```bash
# 终端 1:硬件 + 定位(FAST-LIO / MAVROS;C 区不需相机)
/home/cwkj/competition_ws_school/start_hardware_and_camera.sh

# 终端 2:C 区避障测试(实飞,不开 RViz)
./run_c_zone.sh          # 默认 cruise_height=1.8
# ./run_c_zone.sh 1.5    # 自定义限飞高度(米),务必 <2.0

# 终端 3:录包(可选,手动按需 —— 不是每次飞都录;Ctrl-C 停)
./record_bag.sh          # 录到 logs/c_zone_<时间戳>.bag,开录前自动检查话题
```

`run_c_zone.sh` 把"建日志目录 + 启动 + 终端日志留底"打包成一条命令,**实飞默认不开 RViz**
(无线连接传点云会非常卡;RViz 留给离线复盘)。

> 📁 飞行日志自动落到工作空间内的 `logs/`(随空间走、重启不丢):
> - **CSV**(`logs/flight_*.csv`):节点自动写,记位姿/速度/最近障碍距离/限速系数/关键事件——查"撞没撞、卡在哪"(量)。
> - **终端日志**(`logs/terminal_*.log`):脚本 `tee` 自动留底,记 planner 聚类点数/膨胀值/无解原因——查"为什么"(因)。
> - 两份对照看;详见 [`docs/C_zone_debug.md`](docs/C_zone_debug.md)。

**离线复盘**(回放 rosbag 时才开 RViz,数据走本地不卡):

```bash
roslaunch safe_landing safe_landing.launch rviz:=true
```

> ⚠️ 限高:C 区限高靠 `cruise_height`(建议 ≤1.8m)+ 起飞高度保证,
> **不是** `planner.obstacle_max_z_agl`(那只是障碍点过滤上限)。
> 独立模式末尾会自动评估降落区并 `AUTO.LAND`;只想验证"钻过树林"时,飞过判分线后用 RC 接管即可。

## 调试

调参/排障流程见 [`docs/C_zone_debug.md`](docs/C_zone_debug.md):
**先录包 + 离线复盘**把"定位 + 点云"锁死,再调避障参数。

## 后续计划(TODO)

- [ ] 草拟 C 区特化配置 `config/safe_landing_C.yaml`(限高 1.8、入口→判分线航点、聚类/膨胀裕度)
- [ ] 离线喂 `grid_planner` 的小测试节点,配合录的包扫参数
- [ ] 调好的参数回流到原空间的国赛任务表(`NATIONAL_PLAN.md` 阶段 2a)
