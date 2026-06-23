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
./run_c_zone.sh          # 默认 cruise_height=1.0
# ./run_c_zone.sh 1.5    # 自定义限飞高度(米),务必 <2.0

# 终端 3:录包(可选,手动按需 —— 不是每次飞都录;Ctrl-C 停)
cd /home/cwkj/competition_C_ws
./record_bag.sh          # 录到 logs/c_zone_<时间戳>.bag,开录前自动检查话题
```

`run_c_zone.sh` 把"建日志目录 + 启动 + 终端日志留底"打包成一条命令,**实飞默认不开 RViz**
(无线连接传点云会非常卡;RViz 留给离线复盘)。

### 启动后完整推演(怎么启动 → 怎么动 → 结束得到什么)

> 基于当前仓库真实配置:`run_c_zone.sh` 默认 `cruise_height=1.0`、`safe_landing.yaml` 航点 `x=3.4`、
> `local_map` 默认开。以下是**基于代码的推演,尚未实飞验证**。

#### 启动瞬间(还没动)

`./run_c_zone.sh` 一跑,起两个节点 + 建两份日志,终端大致刷出(措辞与代码一致):

```
[INFO] local_obstacle_map: /cloud_registered (+/Odometry) -> /cloud_local_map | max_age=1.50s max_range=6.0m ...
[INFO] FlightLogger: recording to /home/cwkj/competition_C_ws/logs/flight_<时间戳>.csv
[INFO] safe_landing: loaded 1 waypoints
[INFO] safe_landing_node up: cruise_h=1.00, 1 waypoints, marker=off
[INFO] init pose: (0.0, 0.0, 0.0) yaw=...      ← 此刻位姿被钉为坐标原点,机头朝向 = +x
```

此时:**局部地图节点**开始把 `/cloud_registered` 累积成 `/cloud_local_map`;**主节点**在等里程计稳定
(`WaitOdom`)。CSV / 终端日志已在 `logs/` 里逐帧写入。

#### 无人机运动时间线

| 阶段 | 物理动作 | 终端打印(示意) | CSV 里 |
|---|---|---|---|
| **Takeoff** | 原地解锁、进 OFFBOARD,**垂直爬到 1.0m** 悬停 | `OFFBOARD+armed` → `takeoff complete (z err..)` | stage=Takeoff,z:0→1.0 |
| **Cruise** | 机头朝 +x,**平飞奔向 3.4m 外目标**;遇圣诞树自动减速/侧绕,绕完回正继续 | `[wp 1/1] target=(3.4,0,..)`、`planner: ... clusters=N kept_points=..`、`planner: path cells=..`;有树时 `[wp1] blocked..`→绕过→`[wp1] reached (d=..)` | stage=Cruise,`d_fwd/d_side` 实时变,绕树处 `scale<1`、`event=blocked/replan/skip` |
| **Hover** | 到目标上方**悬停**片刻 | (稳定保持) | stage=Hover |
| **Evaluate** | 不动,**用点云在正下方找平整安全地面** | `landing zone OK at (..) slope=.. rough=..` | stage=Evaluate |
| **MoveToSafe** | **平移到选中的安全点上方**(通常就在附近) | `move-to-safe done (d=..), descending` | stage=MoveToSafe |
| **Descend** | **缓慢下降**,边降边查下方障碍(异常则减速/中止重评估) | `descent: reached target AGL 0.20 m` | stage=Descend,`d_fwd`=下方障碍距离 |
| **Precision** | 到 ~0.2–0.3m 交 `AUTO.LAND`,**落地、自动上锁** | `precision: switching to AUTO.LAND` → `vehicle disarmed, landing complete` | stage=Precision→Done |
| **Done** | 停在地面,电机锁定 | `DONE — task finished` | stage=Done |

> 一句话:**原地起飞到 1.0m → 避障平飞 3.4m 到目标 → 在目标上方评估并自动降落上锁,全程无人干预。**
> 只想验证"钻过树林"而不自动降落 → 飞过目标后用 RC 接管。改飞行距离/方向 → 改 `safe_landing.yaml`
> 里 `waypoints` 的 `x/y`(相对原点偏移,m;现 `x=3.4` 偏短,真正穿林需调大)。

#### 可能出现的"非理想现象"(按当前最该警惕排序)

- **遮挡(已缓解,但非万能)**:两棵树严格一条线、迎面直飞时,**前树挡死后树,雷达根本扫不到,记忆也变不出来**。
  后树是在**侧绕前树的过程中**(横向挪开约 0.3–0.5m)才逐渐露出近侧边缘的。局部地图的作用是把这些
  **最早露出的零星稀点累积、攒密并保留 1.5s**,让"知道后树存在"从"绕完正对时"**提前到"侧绕刚开始时"**
  → 多争取的这点时间,就是从"反应不及"到"来得及绕"的差别。**但若共线 + 树缝窄 + 飞得快,仍可能太晚 →
  慢飞是硬要求。这正是本次改动要验证的最难特例。**
- **定位漂移(未解决)**:FAST-LIO 在林中漂得狠时,`/cloud_local_map` 会出现树重影 → A* 可能把通道判死
  (绕外圈/卡住)或飞错位置。局部地图只"框住"危害,没治根。
- **航点偏短**:`x=3.4` 只是往前 3.4m 一跳,目前更像功能验证而非穿越整片林。
- **末尾自动降落**:不想让它自己降时,飞过目标用 RC 接管。

#### 结束后你得到什么

落地上锁后,`logs/` 里多出(同一时间戳):

```
logs/
├── flight_<时间戳>.csv      ← 自动:逐帧位姿/速度/最近障碍距离/限速系数/关键事件(查"撞没撞、卡哪"——量)
├── terminal_<时间戳>.log    ← tee :planner 聚类点数/膨胀/无解原因等终端全文(查"为什么"——因)
└── c_zone_<时间戳>.bag      ← 仅当你另开了 record_bag.sh:原始雷达+点云+位姿,可离线复盘/重跑 FAST-LIO
```

加上**物理结果**:无人机停在目标点(≈原点前 3.4m)附近、已上锁。
复盘:**先看 CSV** 定位哪个阶段哪个量出问题(快),**再用 bag** 回放那一刻看点云/建图(准);详见
[`docs/C_zone_debug.md`](docs/C_zone_debug.md)。

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
