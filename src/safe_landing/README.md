# safe_landing

Obstacle-aware landing for a quadrotor running PX4 + MAVROS + FAST-LIO.

This package handles **cruise + landing**: a waypoint follower with reactive
LiDAR avoidance gets you to the target, then a landing-zone evaluator picks
a flat clear patch, a guarded descent watches for obstacles below, and an
optional AR-tag pass aligns the final touchdown before handing off to PX4
`AUTO.LAND`. No external planner required.

## Pipeline

```
WaitOdom → Takeoff → Cruise (waypoints + reactive avoidance)
        → Hover → Evaluate (RANSAC plane + clearance + grid search)
        → MoveToSafe → GuardedDescent (LiDAR cone, slow/stop/climb-back)
        → PrecisionLanding (optional AR-tag align → AUTO.LAND)
        → Done
```

| Stage             | What it does                                                                 |
|-------------------|------------------------------------------------------------------------------|
| Takeoff           | Streams setpoints, switches OFFBOARD, arms, climbs to `cruise_height` AGL    |
| Cruise            | Walks the waypoint list; speed-capped P controller + forward cone avoidance  |
| Hover             | Settles 1.5 s so a fresh point cloud is available                            |
| Evaluate          | RANSAC plane fit on a 0.4 m disc; checks slope, roughness, clearance         |
| MoveToSafe        | If grid search picked a different XY, slides over to it at cruise altitude   |
| GuardedDescent    | Velocity-mode descent, slows < 0.4 m, hovers < 0.2 m, climbs back if blocked |
| PrecisionLanding  | AR-tag P-control, then `AUTO.LAND` (PX4 handles touchdown + disarm)          |

## Topics

| Direction | Topic                              | Type                              |
|-----------|------------------------------------|-----------------------------------|
| sub       | `/mavros/state`                    | `mavros_msgs/State`               |
| sub       | `/mavros/local_position/odom`      | `nav_msgs/Odometry`               |
| sub       | `/cloud_registered` *(FAST-LIO)*   | `sensor_msgs/PointCloud2` (world frame) |
| sub       | `/ar_pose_marker` *(optional)*     | `ar_track_alvar_msgs/AlvarMarkers`|
| pub       | `/mavros/setpoint_raw/local`       | `mavros_msgs/PositionTarget`      |
| pub (viz) | `/safe_landing/zone_markers`       | `visualization_msgs/MarkerArray`  |

## Build

```bash
cd ~/competition_ws_plus
catkin_make            # or catkin build
source devel/setup.bash
```

If `ar_track_alvar_msgs` isn't installed, the build still succeeds; marker mode
is silently disabled at runtime.

## Run

In four terminals (with `cwkj_ws` and `competition_ws_plus` both sourced):

```bash
# 1) flight stack — your existing launchers
roslaunch robot_bringup px4.launch
roslaunch livox_ros_driver2 msg_MID360.launch
roslaunch fast_lio mapping_mid360.launch

# 2) (optional) EGO-Planner
roslaunch ego_planner single_run_in_sim.launch

# 3) (optional) AR-tag detection
roslaunch ar_track_alvar pr2_indiv.launch

# 4) safe_landing
roslaunch safe_landing safe_landing.launch
```

Tune `config/safe_landing.yaml` — at minimum set `land_x`, `land_y` to the
world-frame target. To enable AR-tag fine alignment, set `use_marker: true`
and `marker_id`.

## Why it lands safely

1. **Plane evaluation, not point-distance heuristics.** RANSAC + RMS residual
   gives slope and roughness in real units. A 5 cm box on flat ground will
   show as roughness, not as "low clearance".
2. **Clearance check is separate from plane fit.** A spike above the patch
   (like a thin pole) wouldn't be a RANSAC inlier, so we explicitly count
   points in a clearance cylinder.
3. **Velocity-mode descent.** Position setpoints would push the drone *into*
   an obstacle waiting for the controller to give up; vz=0 hovers immediately.
4. **PX4 AUTO.LAND for touchdown.** PX4 has tuned land-detector logic; manual
   z setpoints to the floor often cause bounce or pre-disarm yaw kicks.

## Tuning notes

- `zone.max_slope_deg`: 5–8° for well-aligned LIO; bump to 12° if your IMU
  attitude is noisy in indoor environments.
- `zone.max_roughness`: 2–3 cm for a Mid360 with good extrinsics. If you see
  many false rejects on clearly flat ground, raise to 5 cm.
- `descent.cone_half_angle_deg`: 15–25°. Wider catches tilted obstacles
  earlier but admits floor returns at low altitude — drop `target_z` if you
  see false aborts near the ground.
- `descent.target_z`: Don't go below 0.20 m. Below that, LiDAR returns from
  the ground itself trip the proximity guard.

## Known limits

- Trusts FAST-LIO localization; if LIO drifts the cloud disagrees with the
  PX4 odometry, the picked zone will be wrong by the drift.
- World-frame cloud is assumed (`/cloud_registered`). For body-frame clouds,
  remap or transform first.
- Single-vehicle. The `drone_0_*` topics are kept for compatibility with the
  existing `cwkj_ws` configuration.
