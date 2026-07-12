#!/bin/bash
# Build the Nav2 overlay workspace. Intended to be run INSIDE the container:
#   docker compose -f docker/docker-compose.yml exec nav2 src/navigation2/docker/colcon-build.sh
set -eo pipefail

# shellcheck disable=SC1091
source "${UNDERLAY_WS:-/opt/underlay_ws}/install/setup.bash"

cd "${OVERLAY_WS:-/opt/overlay_ws}"

# Refresh dependencies in case the mounted source changed since the image build.
# Skip keys:
#   slam_toolbox                 - not built here (nav2 default upstream skip)
#   realsense2_camera, velodyne,
#   imu_filter_madgwick          - heavy hardware SDKs, only exec-deps of the
#                                  rtabmap_examples sample package (not needed here)
SKIP_KEYS="${SKIP_KEYS:-slam_toolbox realsense2_camera velodyne imu_filter_madgwick}"
apt-get update >/dev/null 2>&1 || true
rosdep install -q -y \
  --from-paths src \
  --skip-keys "$SKIP_KEYS" \
  --ignore-src || true

# Limit build parallelism to avoid the OOM killer on memory-constrained hosts.
# Total concurrent compile jobs ~= PARALLEL_WORKERS * MAKE_JOBS.
# Override via env, e.g. PARALLEL_WORKERS=8 MAKE_JOBS=2 for a larger machine.
PARALLEL_WORKERS="${PARALLEL_WORKERS:-4}"
MAKE_JOBS="${MAKE_JOBS:-2}"
export MAKEFLAGS="-j${MAKE_JOBS}"

# rtabmap_costmap_plugins targets an older nav2 costmap API and does not compile
# against the (rolling-based) nav2 built here; skip it until it is ported.
PACKAGES_IGNORE="${PACKAGES_IGNORE:-rtabmap_costmap_plugins}"

colcon build \
  --symlink-install \
  --parallel-workers "${PARALLEL_WORKERS}" \
  --packages-ignore ${PACKAGES_IGNORE} \
  --mixin ${OVERLAY_MIXINS:-release ccache lld} \
  --event-handlers console_direct+ \
  "$@"
