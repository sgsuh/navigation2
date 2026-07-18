# RTAB-Map localization/mapping system test

Exercises `bringup_launch.py localization:=rtabmap` end to end without a robot,
a physics simulator, or a camera.

## What it does

`nav2_loopback_sim` stands in for the hardware. It raycasts a ground truth map
into a `LaserScan` and integrates `cmd_vel` into the `odom` -> `base_link`
transform, with its own `publish_map_odom_tf` disabled so that **RTAB-Map is the
single owner of `map` -> `odom`** — the same division of labour as a real robot
with wheel odometry.

The ground truth (`nav2_bringup/maps/depot.yaml`) is reachable only through the
`/map_server/map` service and is published on `ground_truth_map`, never on
`map`. RTAB-Map therefore never sees it: it has to rebuild the map from scans
alone.

## What it asserts

1. The mock produces a `LaserScan` with finite returns. This is also the signal
   that `loopback_simulator` has fully initialised — it silently emits an
   all-inf scan until it has the ground truth map, an initial pose, and the
   `base_link` -> `base_scan` transform.
2. RTAB-Map publishes an occupancy grid on `map` with real occupied and free
   cells, in the `map` frame.
3. RTAB-Map publishes the `map` -> `odom` transform, the one AMCL would
   otherwise supply.
4. **Nav2's `StaticLayer` actually consumes that grid**, checked by requiring
   `global_costmap` to resize to the grid's exact dimensions. A non-rolling
   global costmap adopts whatever size `StaticLayer` hands it, so matching
   dimensions can only come from RTAB-Map's map. Counting lethal cells would
   *not* work as a check: the `ObstacleLayer` marks cells straight from `scan`
   and would keep the count high even with `StaticLayer` completely mis-wired.

Check 4 is the one that matters. The failure it guards against — a costmap that
looks populated while silently ignoring the map, because of a topic-namespace or
QoS mismatch — is invisible from `ros2 topic echo /map`.

## Known gaps

- Runs with `use_rgbd:=False`, unlike the default configuration. There is no
  synthetic camera: static fake images would yield meaningless visual features
  and bogus loop closures, so they would prove nothing. The map-topic and QoS
  wiring under test is identical either way, but **the RGB-D path itself is not
  covered here**.
- Mapping mode only. Localization mode against a prebuilt database is not
  exercised.
