# C 区(茂密森林)调试指南

> 任务:无人机自主穿过乱序排列的仿真树(高约 2m、直径约 0.5m),**飞行高度不得超过 2m**(超过该项 0 分),稳定飞过判分线得 100。外部绕行不得分。树木位置赛前随机变更——**不能背航点,必须靠实时定位 + 点云 + 规划/反应避障**。
>
> 本文档给出推荐的调试方法、故障定位流程和常见坑。纯调试参考,不涉及代码改动。

---

## 0. 核心思路

C 区八成的"撞树 / 卡住 / 飘走"最后都追到两个环节:**① 定位漂移** 或 **② 树被点云漏看 / 被聚类滤掉**。
因此最高效的策略是:

1. 用**两个互补的复盘手段**采集证据,**不靠肉眼盯实时**:
   - **录包(rosbag)**——手飞也能录,**开环**复盘定位 / 点云 / 聚类 / 规划,能离线换参数重放(见 §1)。
   - **CSV 飞行日志(flight_logger)**——只在 safe_landing **自主运行**时写,记录避障**当时的决策内幕**(前/侧向障碍距离、限速系数、replan/blocked/skip 事件),这是录包**回放不出来**的闭环信息(见 §2)。
2. 排查时**从底层往上查**(定位→点云→规划→反应避障→控制):底层坏了上层一定乱,反之不成立。在定位还在漂的时候去调避障参数纯属白费。

> 一句话分工:**录包查"看到了什么"(开环),CSV 查"为什么这么动"(闭环)。** 两者结合才能把整条链路覆盖全。

---

## 1. 手段一:录包(rosbag,开环复盘)

### 1.1 录包

录包**只依赖硬件/定位层**(终端 1),不会解锁、不会起飞,所以**不飞也能录**。两种采集方式:

- **地面手持(推荐先做,零风险)**:`start_hardware_and_camera.sh` 起来后,**手端无人机慢慢走过/绕过树**,
  再走回起点。能验证:传感器在出数、FAST-LIO 漂不漂、点云有没有扫到树干。
- **手动飞**:飞手用 RC(POSCTL/定点)手动穿一遍(不开自主避障),拿到真实飞行下的点云。

两种都用同一条录包命令:

```bash
# 终端 1:先起硬件 + 定位(不解锁)
/home/cwkj/competition_ws_school/start_hardware_and_camera.sh

# 终端 2:录包
cd /home/cwkj/competition_C_ws && ./record_bag.sh     # → logs/c_zone_<时间戳>.bag,Ctrl-C 停
```

`record_bag.sh` 已封装下面这条:自动建目录、带时间戳存到 `logs/`、开录前检查话题在不在。

```bash
rosbag record -O c_zone_$(date +%H%M%S).bag --lz4 \
  /livox/lidar /livox/imu \                    # 原始雷达 + IMU:可离线重跑 FAST-LIO、换参数
  /Odometry \                                  # FAST-LIO 里程计
  /cloud_registered \                          # 世界系点云:规划/避障/降落区评估直接吃这个
  /mavros/local_position/odom \
  /mavros/vision_pose/pose \                   # lidar_bridge 喂给飞控的位姿
  /mavros/state \
  /mavros/setpoint_raw/local \                 # 手飞时为空;自主时记录控制指令
  /tf /tf_static
```

> - 话题名以实际 `rostopic list` 为准(雷达可能是 `/livox/lidar` 或其它命名),缺哪个脚本会提示。
> - **录之前先 `rostopic hz <话题>` 确认有数据再开录**,别录空包。
> - `/cloud_local_map` **不用录**(派生话题),离线回放时由 `local_obstacle_map_node` 现算(见 §1.2)。

**为什么 raw + registered 都录(决定下面两种回放都能用):**
- `/cloud_registered`:离线复盘规划/聚类(FAST-LIO 配置已固化进点云)。→ 喂**模式一**。
- `/livox/lidar` + `/livox/imu`:若怀疑定位本身,可离线**换 FAST-LIO 参数重跑**。→ 喂**模式二**。

### 1.2 两种回放模式(录什么决定能看什么)

同一个包,有**两种**放法,看到的东西完全不同——别混:

|            | 模式一:回放「 FAST-LIO的输出」           | 模式二:回放「雷达/imu原始数据」 |
|---         |--                                      -|-                                       -|
| 放什么     | `/cloud_registered`、`/Odometry`(成品) | 只放 `/livox/lidar`、`/livox/imu`(原料) |
| FAST-LIO   | **不跑**,只看当时算好的结果            | **重新启动、现场重建图** |
| 能看到     | 漂没漂、怎么漂(成品里已"焊死")          | **建图全流程** + 漂移如何累积 |
| 能改参数吗 | ❌ 改也没用,只是放录像                 | ✅ 改 `mid360.yaml` 重放同包,反复调到不漂 |
| 用途       | 快速看一眼                             | **修漂移的正路** |

#### 模式一:看成品(快,一条命令)

```bash
cd /home/cwkj/competition_C_ws
./replay_bag.sh 包名             # 指定某个包(.bag 后缀可省，不用加地址)
```

`replay_bag.sh` 一条命令把整套起好:**source → 起 roscore → `use_sim_time` → 后台拉起
`local_obstacle_map_node`(现算 `/cloud_local_map`)和 RViz → 前台 `--loop` 循环放包**。
控制:**空格=暂停/继续,Ctrl-C=停止并自动收尾**(连带关掉 RViz / 节点 / 自动起的 roscore)。

<details><summary>等价的手敲命令(脚本内部做的事)</summary>

```bash
roscore & sleep 2
rosparam set use_sim_time true
rosrun safe_landing local_obstacle_map_node \
  _input_topic:=/cloud_registered _output_topic:=/cloud_local_map _odom_topic:=/Odometry &
rviz -d $(rospack find safe_landing)/rviz/safe_landing.rviz &
rosbag play --clock --loop c_zone_xxx.bag
```
</details>

> ⚠️ **RViz 全黑必查**:左上 `Fixed Frame` 必须是 **`camera_init`**(FAST-LIO 的世界系),
> 不是 `map`。手动加点云用 `Add → By topic`:`/cloud_registered`、`/cloud_local_map`、`/Odometry`。
> 加好后 `File → Save Config` 覆盖 `safe_landing.rviz`,下次 `-d` 直接可用。

> ⚠️ **包只有几十秒**:放完话题就消失、RViz 变空。用 `replay_bag.sh`(已带 `--loop` 循环)即可。

**两个点云的区别(避障到底吃哪个):**

| | `/cloud_registered` | `/cloud_local_map` |
|---|---|---|
| 来源 | FAST-LIO 直接发 | `local_obstacle_map_node` 现算(派生话题,**包里没录,需节点在跑**) |
| 内容 | **当前单帧**,稀疏、无记忆 | 最近 **1.5s** 累积,**更密、有记忆、按距离/时间裁剪** |
| 谁用 | 数据源 | **规划/避障真正消费的就是它**(launch 已把 `cloud_topic` 指向它) |

RViz 里 `/cloud_local_map` 应**明显比** `/cloud_registered` 密且稳——这就是修"两棵树一条线避不开"
(单帧稀疏 + 树被遮挡漏点)的关键证据。**这个话题为空 = 节点没起来。**

#### 模式二:重跑 FAST-LIO 看建图全流程(本项目已核实的命令)

本项目链路:`hardware_bringup.launch` 里 **Mid360 驱动** 和 **FAST-LIO** 是分开的两个 include;
FAST-LIO 用 `mapping_mid360.launch`(加载 `config/mid360.yaml`),订阅 **`/livox/lidar` + `/livox/imu`**,
`/livox/lidar` 是 Livox CustomMsg、10Hz。所以模式二 = **只起 FAST-LIO、不起驱动**,把原始话题喂进去:

```bash
# --- 终端 A:只起 FAST-LIO(千万别起 Mid360 驱动,也别用 start_hardware_*.sh)---
source /opt/ros/noetic/setup.bash
source /home/cwkj/ws_livox/devel/setup.bash
source /home/cwkj/fast_lio2_ws/devel/setup.bash
roscore & sleep 2
rosparam set use_sim_time true                # 必须在启动 FAST-LIO 之前设
roslaunch fast_lio mapping_mid360.launch rviz:=true

# --- 终端 B:只回放原始输入(别把 FAST-LIO 的输出也放出来)---
source /opt/ros/noetic/setup.bash
source /home/cwkj/fast_lio2_ws/devel/setup.bash
cd /home/cwkj/competition_C_ws/logs
rosbag play --clock c_zone_xxx.bag --topics /livox/lidar /livox/imu
```

**模式二的三个必踩的坑:**
1. **不要起 Mid360 驱动**(`msg_MID360.launch`)、不要用 `start_hardware_and_camera.sh`(它会把驱动也拉起来)。
   包里已有 `/livox/lidar`,再开实物驱动会双重发布、打架。
2. **`rosbag play` 必须 `--topics /livox/lidar /livox/imu`**,只放原料。整包放会让包里的**旧输出**和
   FAST-LIO **新算的输出**在 `/Odometry`、`/cloud_registered` 上撞车,RViz 全乱。
3. **`use_sim_time true` 要在 FAST-LIO 启动前设**,`play` 带 `--clock`,否则时间戳对不上,建图直接崩。

### 1.3 怎么"一眼看出"FAST-LIO 漂移(回放时)

不管模式一还是模式二,RViz 里把 `/cloud_registered` 的 **Decay Time(衰减时间)调到很大**(如 1000),
让所有帧**叠加**显示:

- **不漂** → 同一棵圣诞树多次扫到 **叠成一个清晰锥体**,墙是单层。
- **漂** → 同一棵树 **散成两三个重影 / 糊成一团**,墙变"双层墙"。这就是漂移铁证。

另外两个补充判据:
- **静止测试**:飞机不动,`rostopic echo` 看 `/Odometry` 的 x/y/z 会不会自己缓慢爬 → 爬就是漂。
- **回环测试**:手推飞机走一圈回到原点,看 odom 终点与起点差多少 → 差很多 = 累积漂移。

### 1.4 ⚠️ 能查什么 / 不能查什么(关键认知)

**✅ 录包离线能搞定(开环,覆盖约 80% 的坑):**
- 定位漂不漂(replay 看建图有无重影/糊)
- 树是否都进了 `/cloud_registered`(漏检 → 会撞)
- 细树干会不会被**聚类过滤**掉(离线反复调 `cluster_min_points`)
- 给定起点/终点,A* 能否从树缝穿过(离线调 `inflation_radius`)
- 以上都能 **改参数 → 重放同一个包 → 立刻看结果**,不用再飞

**❌ 录包离线查不了(闭环):**
- **完整自主避障行为**:包里的轨迹是手飞那次的。改了规划参数后,飞机本该飞到不同位置、看到不同点云,这个闭环无法靠回放重现。回放只能回答"在这个位置、这帧点云下规划器输出啥",不能回答"换参数后整条航线会怎样"。
- 控制跟踪 / OFFBOARD 行为

### 1.5 两个层次

- **眼睛级(零成本,最值)**:replay + RViz 肉眼看定位与点云,挡掉"定位漂 / 树漏检 / 坐标错"。
- **代码级(要点功夫)**:写个小测试节点订阅包里的 `/cloud_registered`,单独喂 `grid_planner` 的聚类 + A*,离线扫 `inflation_radius` / `cluster_min_points` 找最优。

### 1.6 实用提醒
- 点云包很大(几十秒可达几百 MB~GB):用 `--lz4` 压缩、`--split --size=1024` 分卷、只录关键时段。

---

## 2. 手段二:CSV 飞行日志(flight_logger,闭环决策内幕)

safe_landing 内置 `flight_logger`,**自主运行时**会把每帧的状态和避障决策写进
`competition_C_ws/logs/flight_YYYYmmdd_HHMMSS.csv`(目录由 `safe_landing.yaml` 的 `log_dir` 指定，
已设为工作空间内的 `logs/`，随空间走、重启不丢；终端日志也在同目录）。
每行 `flush` 落盘,**即使坠机/硬杀也能保住**。

> 与录包的根本区别:CSV 记的是 **safe_landing 自己当时算出来的避障量**——
> 前/侧向障碍距离、限速系数、为什么停/绕/跳。这些**录包回放不出来**(回放只有传感器,没有当时的决策)。
> 所以 CSV 只在**自主飞行 / SITL** 时有内容,手动 RC 飞行时是空的(那时 safe_landing 没在控制)。

### 2.1 CSV 字段(固定表头)

```
t_rel, t_ros, stage, event, x, y, z, yaw, vx, vy, vz, speed_xy, d_fwd, d_side, scale, goal_x, goal_y
```

| 列 | 含义 | 看它干嘛 |
|---|---|---|
| `t_rel` / `t_ros` | 相对/ROS 时间戳 | 对齐事件、画曲线 |
| `stage` | 状态机阶段(Takeoff/Cruise/Hover/Evaluate/Descend...) | 卡在哪个阶段 |
| `event` | 标签事件(阶段切换、`replan`、`blocked`、`skip`、`abort` 等) | **为什么动作变了** |
| `x,y,z,yaw` | 位姿 | 轨迹、**高度是否 >2m**(看 z) |
| `vx,vy,vz,speed_xy` | 速度 | 是否真的在动 / 悬停 |
| `d_fwd` | 前向锥最近障碍距离(巡航);**下降阶段是下方障碍距离** | 撞没撞、停得对不对 |
| `d_side` | 侧向球最近障碍距离(巡航) | 两树间是否蹭 |
| `scale` | 避障限速系数 0~1(**0=被判阻塞停住**,1=全速) | **为什么不走/变慢** |
| `goal_x,goal_y` | 当前目标航点 | 在朝哪飞 |

> 空单元格 = NaN(非该阶段的列留空),`pandas.read_csv` 直接能读。

### 2.2 怎么用(pandas 三板斧)

```python
import pandas as pd
df = pd.read_csv("/home/cwkj/competition_C_ws/logs/flight_XXXX.csv")

# 1) 撞没撞 / 蹭没蹭:巡航段最近障碍距离的最小值
cruise = df[df.stage == "Cruise"]
print("min d_fwd =", cruise.d_fwd.min(), " min d_side =", cruise.d_side.min())

# 2) 在哪卡住:scale 掉到 ~0 的时刻和位置
stuck = cruise[cruise.scale < 0.05][["t_rel", "x", "y", "d_fwd", "d_side"]]
print(stuck)

# 3) 发生了什么:所有标签事件(replan / blocked / skip / abort / 阶段切换)
print(df[df.event != ""][["t_rel", "stage", "event", "x", "y"]])

# 4) 限高自查:全程最高点
print("max z =", df.z.max())
```

### 2.3 典型诊断模式

| CSV 现象 | 含义 / 下一步 |
|---|---|
| `d_fwd` 或 `d_side` 最小值 ≈ 0(甚至负余量) | **蹭/撞了**;拉开 `stop_dist`/`side_radius` 与膨胀的裕度 |
| `scale` 长时间 = 0,且 `d_fwd` 不算近 | 误判阻塞;检查 `emergency_dist`/`stop_dist` 是否过大,或点云有近距噪点 |
| 频繁 `event=replan` / `scan` 后 `abort`/`skip` | A* 反复无解;多半膨胀太大树缝过不去,调小 `inflation_radius` |
| `event=skip` 某航段 | 该段被判阻塞跳过(`skip_blocked_leg=true`);看是不是树挡死了 |
| `z` 曾 > 2.0 | **C 区会 0 分**;降 `cruise_height`、查起飞过冲 |
| `stage` 一直停在 `Hover`/`Evaluate` | 点云不足或降落区评估失败(C 区可忽略,飞过判分线后 RC 接管) |

### 2.4 录包 + CSV 配合用

**自主试飞时两个一起开**:录包留传感器现场(可离线重放),CSV 留决策内幕。
出问题时:先看 **CSV** 定位是哪个阶段、哪个量触发的(快);再用**录包**回到那一刻看点云/定位长啥样、离线调参验证(准)。

---

## 3. 依赖链与分环节排查

```
① 定位(FAST-LIO /Odometry → /mavros/vision_pose)
   ↓ 飞控有位置估计
② 点云(/cloud_registered,世界系)
   ↓ 障碍被看到
③ 规划(grid_planner A*:点云→栅格→膨胀→路径)
   ↓ 给出绕树航点
④ 反应避障(waypoint_follower:前向锥/侧向球→减速/停)
   ↓ 速度指令
⑤ 控制(/mavros/setpoint_raw/local → PX4 OFFBOARD 跟踪)
```

### ① 定位(C 区头号杀手)
```bash
rostopic echo -c /mavros/local_position/odom/pose/pose/position   # 静止时 x/y/z 应基本不变
rostopic hz /Odometry                # FAST-LIO 输出频率应稳定
rostopic hz /mavros/vision_pose/pose # 位姿是否喂给飞控
```
- RViz 加载 `safe_landing.rviz`,手推飞机走一圈,点云地图**不应重影/漂移**。
- 判据:静止漂 > 几 cm/s,或飞行中地图糊 → 定位问题,**先停在这,别往下查**。
- **怀疑漂移就走录包正路**:录原始 `/livox/lidar`+`/livox/imu`,用 **§1.2 模式二**重跑 FAST-LIO、
  按 **§1.3** 的 Decay Time 叠加法看重影,并可改 `mid360.yaml` 参数重放同包反复调。

### ② 点云
```bash
rostopic hz /cloud_registered        # 有无、频率稳否
```
- RViz 看 `/cloud_registered`:树是否都成点簇?该有树处是否空了(漏检→撞)?是否满屏噪点(误检→卡死)?

### ③ 规划
- safe_landing 节点终端日志看 `plan` / `replan` / `scan` / `failed`。
- 反复 `replan` / `scan yaw` / `planner failed` → A* 无解(多半膨胀太大,树缝过不去)。
- RViz 看规划路径:从树**之间**穿,还是绕**外圈**(绕外不得分)。

### ④ 反应避障
```bash
rostopic echo /mavros/setpoint_raw/local   # 看速度指令
```
- 树前**莫名悬停** → 反应层判阻塞(前向 < `stop_dist` / `emergency_dist`)。日志看 `blocked` / `abort`。
- 关键参数:`stop_dist 0.33` / `emergency_dist 0.28` / `side_radius 0.28`。

### ⑤ 控制
```bash
rostopic echo /mavros/state          # mode 是否 OFFBOARD、armed 是否 true
```
- 指令正常但飞机不跟 → OFFBOARD 没进/被踢,或 setpoint 流断。

### 贯穿全程的复盘工具
自主飞行时看 **CSV 飞行日志**(详见 §2):`scale`、`d_fwd`/`d_side`、`event` 直接告诉你
"在哪卡的、为什么停的、撞没撞"。手动飞行时则靠**录包 + RViz**(§1)看定位与点云。

---

## 4. C 区最可能踩的坑(按概率排序)

| # | 症状 | 最可能原因 | 怎么确认 / 处理 |
|---|---|---|---|
| 1 | 飞行中位置漂、地图糊 | **FAST-LIO 定位漂移**(林中特征重复) | 静止看 odom 漂移;RViz 看建图重影 |
| 2 | **撞树**但日志没报障碍 | 细树干点太少被**聚类过滤**当噪点(`cluster_min_points=8` 偏高) | RViz 看那棵树是否在点云里;调到 3~5 复测 |
| 3 | 树前**卡住**最后超时 | 膨胀太大,树缝 < 2×膨胀,A* 无解 | 日志反复 `replan/scan/failed`;调小 `inflation_radius` |
| 4 | **绕外圈**飞(不得分) | 同上,规划判内部不可通行 | RViz 看路径绕外 |
| 5 | C 区判 **0 分**但没撞 | **飞行高度 > 2m**(cruise_height 设高 / 起飞过冲) | 全程盯 odom 的 z;`cruise_height` 设 ≤1.8 |
| 6 | 两树之间**蹭过去** | `stop_dist`==膨胀无裕度,或 `side_radius` 偏小 | CSV 看最近障碍距离是否触 0;拉开裕度 |
| 7 | 起飞就横飘 / 进不了 OFFBOARD | 定位没喂飞控 / setpoint 流问题 | 查 `/mavros/vision_pose/pose`、`/mavros/state` |
| 8 | 障碍出现在**错误位置** | 点云坐标系与 odom 不一致(TF/frame) | RViz 里点云与飞机模型对不上 |

> ⚠️ 限高提醒:C 区限高靠 `cruise_height`(建议 ≤1.8m)+ 起飞高度保证。
> `planner.obstacle_max_z_agl` 只是"参与规划的障碍过滤上限",**不是飞行限高**,别指望它。

---

## 5. 推荐调试流程(省时间)

1. **录包 + 离线复盘**(§1):手飞一趟录包,replay + RViz 先把 ① 定位、② 点云锁死。这步挡掉一半问题且零风险。
2. **地面手推验证**:不起飞,手推飞机走 C 区一圈,确认定位不漂 + 树都成点云。
3. **空场试飞**:无树,只验证起飞→巡航→降落链路(③④⑤控制正常)。
4. **逐步加树**:从少量树、宽间距开始逐步加密,**每次只调一个参数**(先膨胀/聚类,再裕度)。
5. 每飞一次都存 CSV,**先看 CSV 再改参数**,别凭感觉调。
6. 调好后,把这套参数固化为分区配置 `safe_landing_C.yaml`(供后续国赛任务表使用)。

---

## 6. 一句话总结

C 区调试的命门是 **① 定位 + ② 点云**这两环。**先用"录包 + 离线复盘 + RViz"把这两环在真实树林数据上验证透,再谈调避障参数**——否则地基没夯实,上层怎么调都是错的。
两个证据源配合:**录包查"看到了什么"(开环),CSV 查"为什么这么动"(闭环)。**
