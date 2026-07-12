# Nav2 ROS 2 Jazzy Docker Environment

A container environment to build and develop this Nav2 workspace on **ROS 2 Jazzy**.
Everything (dependencies, colcon build, runtime) happens inside the container; the
host workspace is only bind-mounted as source.

## Layout

| File | Purpose |
| --- | --- |
| `docker-compose.yml` | Service definition. Reuses the root `Dockerfile` (`dever` target) with `FROM_IMAGE=ros:jazzy` and the Jazzy underlay. |
| `build.sh` | Build the image. |
| `into.sh` | Start the container and open a shell. |
| `colcon-build.sh` | Build the overlay (run inside the container). |

The service uses two named volumes so build artifacts stay inside the container:
- `nav2_overlay` → `/opt/overlay_ws` (build/install/log)
- `nav2_ccache` → `/tmp/.ccache` (compiler cache)

The source tree is bind-mounted at `/opt/overlay_ws/src/navigation2`.

## Quick start

From the repository root:

```bash
# 1. Build the image (underlay + overlay dependencies)
./docker/build.sh

# 2. Start the container and enter a shell
./docker/into.sh

# 3. Inside the container, build the workspace
src/navigation2/docker/colcon-build.sh
```

Or without the shell:

```bash
docker compose -f docker/docker-compose.yml up -d
docker compose -f docker/docker-compose.yml exec nav2 \
  src/navigation2/docker/colcon-build.sh
```

## RTAB-Map integration

RTAB-Map is vendored as git submodules under `third_party/` (branch
`feature/2d-lidar-rgbd-camera`), built into the same overlay:

- `third_party/rtabmap` — core library (v0.23.x from source; the Jazzy debian is only
  0.22.1, too old for the wrapper).
- `third_party/rtabmap_ros` — ROS wrapper (2D lidar + RGBD camera feature).

After a fresh clone, initialize them with:

```bash
git submodule update --init --recursive
```

Build exclusions are configured in the build tooling (the submodules stay pristine):

- `rtabmap_costmap_plugins` is **not built** — it targets an older nav2 costmap API and
  does not compile against the rolling-based Nav2 built here. It is skipped via
  `--packages-ignore` in `colcon-build.sh` (override with `PACKAGES_IGNORE=...`). Porting
  it is deferred to the Nav2 integration phase.
- Heavy hardware SDKs that are only exec-deps of the `rtabmap_examples` sample package
  (`realsense2_camera`, `velodyne`, `imu_filter_madgwick`) are skipped by rosdep via
  `SKIP_KEYS` (colcon-build.sh) / `OVERLAY_SKIP_KEYS` (docker-compose.yml, at image build).

> Note: do **not** add `COLCON_IGNORE` markers inside the rtabmap submodules — that hides
> `rtabmap_costmap_plugins` from rosdep, which then pulls the debian rtabmap stack and
> shadows the source build.

## Notes

- The underlay (BehaviorTree.CPP pinned in `tools/underlay.jazzy.repos`) is built into
  the image. Overlay dependencies are resolved with `rosdep` at image build time.
- To rebuild only changed packages, pass extra colcon args, e.g.
  `colcon-build.sh --packages-select nav2_costmap_2d`.
- Stop / remove: `docker compose -f docker/docker-compose.yml down`
  (add `-v` to also drop the build/ccache volumes).
