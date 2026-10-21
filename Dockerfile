# Pinned digest of ros:humble-ros-base-jammy (2026-10-21): rebuilds get the same apt packages. Bump deliberately.
FROM ros:humble-ros-base-jammy@sha256:566b87e6dfc48392189a9c32e594e84060fa9948de93b04e536f39475a1f6963
COPY --from=ghcr.io/astral-sh/uv:0.9.5 /uv /uvx /bin/
RUN apt-get update && apt-get install -y --no-install-recommends \
      git cmake build-essential python3-dev libegl1 libgl1 libosmesa6 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Everything this image provides lives in /opt/robot-learning; /workspace is left for the consuming project.
WORKDIR /opt/robot-learning
COPY pal_mjlab pal_mjlab
COPY LIBERO LIBERO

# Both venvs use ROS Humble's system Python 3.10 (never a uv-downloaded one).
ENV UV_PYTHON=/usr/bin/python3.10 UV_PYTHON_DOWNLOADS=never

# Venv 1: pal_mjlab / mjlab 1.6.0, from PAL's lockfile. cu128 or cpu.
# --system-site-packages (as ROS 2 docs advise for venvs): rclpy and ROS's pytest plugins need ROS's apt Python deps.
ARG TORCH_EXTRA=cu128
RUN uv venv --system-site-packages /opt/venvs/mjlab \
    && UV_PROJECT_ENVIRONMENT=/opt/venvs/mjlab uv sync --frozen --project pal_mjlab --extra ${TORCH_EXTRA} \
    && uv pip install --python /opt/venvs/mjlab pytest

# Venv 2: original LIBERO, its own pinned requirements (numpy 1.22 clashes with mjlab's numpy 2, so never one venv).
# mujoco pinned to the version robosuite 1.4.0 is known to work with. compat editable: LIBERO has no top-level __init__.
RUN uv venv /opt/venvs/libero \
    && uv pip install --python /opt/venvs/libero -r LIBERO/requirements.txt mujoco==3.2.3 \
       -e LIBERO -C editable_mode=compat

# LIBERO calls input() on first import unless this config exists. Outside /root, so any --user can read it.
RUN P=/opt/robot-learning/LIBERO/libero/libero && mkdir -p /etc/libero \
    && printf "benchmark_root: $P\nbddl_files: $P/bddl_files\ninit_states: $P/init_files\ndatasets: $P/../datasets\nassets: $P/assets\n" \
       > /etc/libero/config.yaml && mkdir -m 777 -p /cache/warp

# The seated scene package, editable so a repo mounted over /opt/robot-learning is live; deps come from the venv.
COPY assets assets
COPY pyproject.toml README.md ./
COPY kangaroo_seated kangaroo_seated
COPY tests tests
RUN uv pip install --python /opt/venvs/mjlab --no-deps -e .

# The base image's entrypoint sources ROS Humble for every command, including this bash.
# ROS Humble's launch_testing pytest plugins (on the sourced PYTHONPATH) use a hook pytest 8 removed: off for every project.
# Runs as root or as the host user (--user, no passwd entry): HOME, USER (torch asks getpass), the LIBERO config and
# the Warp kernel cache work for both.
ENV PATH=/opt/venvs/mjlab/bin:$PATH MUJOCO_GL=egl PYOPENGL_PLATFORM=egl NVIDIA_DRIVER_CAPABILITIES=all \
    PYTEST_ADDOPTS="-p no:launch_testing -p no:launch_ros" \
    HOME=/tmp USER=developer LIBERO_CONFIG_PATH=/etc/libero WARP_CACHE_PATH=/cache/warp
WORKDIR /workspace
CMD ["bash"]
