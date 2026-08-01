# RTAB-Map localization/mapping system tests

Three tests. The first two exercise `bringup_launch.py localization:=rtabmap`
end to end without a robot, a physics simulator, or a camera. The third covers
the RGB-D path the other two cannot, and needs Gazebo to do it.

| Test | Mode | What it covers |
|---|---|---|
| `test_rtabmap` | `rtabmap_mode:=mapping` | builds a map from scratch, leaves the database behind |
| `test_rtabmap_localization` | `rtabmap_mode:=localization` | re-uses that database read-only |
| `test_rtabmap_visual` | `rtabmap_mode:=mapping`, `use_rgbd:=True` | visual loop closure from a rendered RGB-D camera |

`test_rtabmap` declares `FIXTURES_SETUP rtabmap_database` and the localization
test declares `FIXTURES_REQUIRED`, so the second only runs after the first has
passed, and never concurrently with it — they share one database file.
`test_rtabmap_visual` shares nothing with either and is independent of both.

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

Both tests must publish an initial pose before anything happens at all:
`loopback_simulator` creates its scan and odom timers inside the `/initialpose`
callback, so until a pose arrives it publishes nothing — not even the all-inf
scan it falls back to when it cannot raycast. Nothing else in either launch
publishes that topic, so **every wait for a scan has to be pumping the pose
itself**; one that does not can never be satisfied. `test_rtabmap` had exactly
that bug and spent a fixed 60 s in it on every run until 2026-08-01, which was
two thirds of its runtime.

## What test_rtabmap asserts

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

## What test_rtabmap_localization asserts

1. `Mem/IncrementalMemory` reads back as `"false"` off the running node — the
   one parameter that distinguishes the two modes, and proof that
   `rtabmap_mode:=localization` selected `rtabmap_localization.yaml`.
2. A map with real occupied cells is published. Since the graph cannot grow in
   this mode, such a map has nowhere to come from except the database. Note
   the robot has to be nudged first: RTAB-Map publishes the grid from its
   processing loop, so a standing robot publishes nothing even with the
   database already loaded.
3. `map` -> `odom` is published, as in mapping mode.
4. `StaticLayer` consumes the stored map, same dimension check as above.
5. Driving does not change the grid — dimensions *and* occupied-cell count
   stay identical. The count is what gives this teeth: a short drive through
   already-mapped space need not enlarge the bounding box even when mapping is
   on, but integrating new scans would move cells.

## What test_rtabmap_visual asserts

This is the only test here that runs in Gazebo, because it is the only one that
needs pixels. It drives three laps of a racetrack — two straights joined by two
180° turns, so each lap comes back to the start pose *at the same heading* —
and asserts:

1. RTAB-Map extracts a useful number of visual words per frame (median ≥ 10;
   it measures ~67).
2. It reports at least 5 **`loop_closure_id`** detections (it measures 40–49).

`loop_closure_id` is the assertion that matters. RTAB-Map's global loop closure
detector is appearance-based — a Bayes filter over a bag of visual words — so it
can only fire when images actually reach the feature extractor. Verified against
an inverted setup: the identical run with `use_rgbd:=False` reports 0 words and
0 loop closures and the test exits 1, while proximity detections stay at ~100.

**`proximity_detection_id` is deliberately not asserted on.** That detector runs
off the lidar and fires either way, so a test keyed on it would pass with the
camera completely dead.

Three choices worth knowing about:

- **turtlebot4, not the waffle.** tb4's standard description already has an
  `rgbd_camera` and `spawn_tb4.launch.py` already bridges colour, depth and
  camera_info. The tb3 waffle's camera is depth-only and its bridge omits the
  camera entirely, so using it would mean committing a modified copy of the
  500-line waffle SDF plus a bridge launch into this package.
- **`worlds/rtabmap_textured_room.sdf`, not tb3_sandbox or depot.** tb3_sandbox
  ships no texture images at all and yields almost no visual features. depot
  works well but is a 101 MB Fuel download — a network dependency at test time
  that nothing else in this package has. The room world gets its features from
  the corners and shading of 70 differently sized crates instead, so it needs no
  external assets. Regenerate it with `worlds/gen_rtabmap_textured_room.py`
  rather than hand-editing 1400 lines of SDF.
- **`rtabmap_launch.py` directly, not `bringup_launch.py`.** The costmap and
  map-topic wiring is already asserted by `test_rtabmap`; bringing up all of Nav2
  here would only add unrelated lifecycle failure modes to a sensor-path test.

It needs a working GL stack, since Gazebo has to render. Software rendering
suffices — `LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe` runs it in real
time at 320x240, ~140 s end to end. If no renderer is available the test fails
at its `camera images` wait rather than somewhere confusing.

## Gotchas found while writing these

- **RTAB-Map segfaults if started in localization mode against an empty or
  missing database** (exit code -11, no diagnostic). Always map first. This is
  upstream behaviour, not something the launch files guard against.
- **`Mem/RawDescriptorsKept` must stay at its default of `true`.** With it set
  to `false`, mapping produces a database that localization mode cannot open at
  all: it aborts on load with `Memory.cpp:526 Condition (!descriptors.empty())
  not met!` (SIGABRT — a different failure from the empty-database segfault
  above). Mapping leaves a handful of dangling word references behind, and
  `Mem/InitWMWithAllNodes` makes a single one of them condemn the whole
  dictionary; RTAB-Map then tries to rebuild it from the per-node descriptors
  that setting suppressed. `rtabmap-recovery` repairs a database already built
  that way.
- **RTAB-Map's `Info` topic is `/info`, not `/rtabmap/info`.** The node
  advertises it relatively and runs in the root namespace. Only
  `/rtabmap/republish_node_data` is genuinely node-scoped. Its statistics live
  in `stats_keys`/`stats_values`, e.g. `Keypoint/Current_frame/words`.
- **`Loop/Visual_inliers/` reads 0 even on a successful visual loop closure.**
  `Reg/Strategy: "1"` selects ICP, so detection is visual but the transform is
  computed from the lidar scans. It is not a sign that the camera is unused.
- **`/localization_pose` is published even when RTAB-Map has not relocalised**,
  because `pub_loc_pose_only_when_localizing` defaults false. Its presence
  proves nothing; check the `map` -> `odom` value instead.
- **A database built in mapping mode carries its parameters with it.** On load
  RTAB-Map logs `Update RTAB-Map parameter "RGBD/LinearUpdate"="0.2" from
  database` and similar, overriding the corresponding values in
  `rtabmap_localization.yaml`. `Mem/IncrementalMemory` still comes from the
  YAML, so the mode itself is unaffected, but the localization-specific
  tuning of `RGBD/AngularUpdate`, `RGBD/LinearUpdate`,
  `RGBD/ProximityPath*` and `Rtabmap/StartNewMapOnLoopClosure` is not what
  ends up running.

## Known gaps

- `test_rtabmap` and `test_rtabmap_localization` run with `use_rgbd:=False`,
  unlike the default configuration: `nav2_loopback_sim` has no camera, and
  static fake images would yield meaningless features and bogus loop closures.
  The map-topic and QoS wiring they cover is identical either way, and the RGB-D
  path is covered separately by `test_rtabmap_visual`.
- **Localization mode is not covered with RGB-D.** `test_rtabmap_visual` maps
  only. Visual relocalisation has been verified by hand (spawned 6.5 m from the
  map origin with no `initial_pose`, it recovers the offset to within 3 cm in
  ~1-3 s, and does not relocalise with the camera off or from an unmapped
  place), but that is not automated here.
- ~~**`test_rtabmap_localization` is intermittently flaky**~~ — **diagnosed
  2026-08-01, and it was not a fault in these tests.** A `change_state` response
  was occasionally dropped by the rmw during bringup, and `lifecycle_manager`
  waited on it with no deadline, so the whole managed set stalled where it stood.
  Whichever node the stall landed on decided which assertion timed out, which is
  why the same bug looked like several different flakes: no usable scan, no
  stored map, or no `costmap_raw`. Fixed by bounding the wait — see the lifecycle
  bringup notes in the repo's `CLAUDE.md` and
  `nav2_lifecycle_manager`'s `test_transition_timeout`. If a run here ever hangs
  in bringup again, grep it for `failed to send response`.
- **A second, unrelated flake is still open**: RTAB-Map loads the database and logs
  `2D occupancy grid map loaded (WxH)` but then publishes no grid, so the map wait
  times out with `RTAB-Map published no usable map` (localization) or `published no
  OccupancyGrid on 'map'` (mapping). Seen with no lifecycle involvement at all — no
  dropped `change_state` reply, no manager error — which is what separates it from
  the bringup deadlock that used to produce the same symptom. The one lead so far is
  a single `Could not convert laser scan msg! Aborting rtabmap update...` a second or
  two before the drive starts. First recorded 2026-07-26, still not diagnosed.

  **Do not try to gate the drive on RTAB-Map's `info` topic.** It was tried on
  2026-08-01 and deadlocks: `info` carries one message per *processed frame*, and
  RTAB-Map does not process frames while the robot is standing still, so waiting for
  it before driving waits forever. Any readiness signal here has to be one that a
  stationary robot can satisfy.

- Localization accuracy is not checked. The tests confirm that `map` -> `odom`
  is published, not that it is correct — with a perfect simulated odometry
  source there is nothing meaningful for the correction to recover from.
