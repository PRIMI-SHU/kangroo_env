FROM ros:humble-ros-base-jammy
COPY --from=ghcr.io/astral-sh/uv:0.9.5 /uv /uvx /bin/
RUN apt-get update && apt-get install -y --no-install-recommends \
      git cmake build-essential python3-dev libegl1 libgl1 libosmesa6 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

COPY pal_mjlab /workspace/pal_mjlab
COPY LIBERO /workspace/LIBERO

# Both venvs use ROS Humble's system Python 3.10 (never a uv-downloaded one).
ENV UV_PYTHON=/usr/bin/python3.10 UV_PYTHON_DOWNLOADS=never

# Venv 1: pal_mjlab / mjlab 1.6.0, from PAL's lockfile. cu128 or cpu.
# --system-site-packages (as ROS 2 docs advise for venvs): rclpy and ROS's pytest plugins need ROS's apt Python deps.
ARG TORCH_EXTRA=cu128
RUN uv venv --system-site-packages /opt/venvs/mjlab \
    && UV_PROJECT_ENVIRONMENT=/opt/venvs/mjlab uv sync --frozen --project /workspace/pal_mjlab --extra ${TORCH_EXTRA} \
    && uv pip install --python /opt/venvs/mjlab pytest

# Venv 2: original LIBERO, its own pinned requirements (numpy 1.22 clashes with mjlab's numpy 2, so never one venv).
# mujoco pinned to the version robosuite 1.4.0 is known to work with. compat editable: LIBERO has no top-level __init__.
RUN uv venv /opt/venvs/libero \
    && uv pip install --python /opt/venvs/libero -r /workspace/LIBERO/requirements.txt mujoco==3.2.3 \
       -e /workspace/LIBERO -C editable_mode=compat

# LIBERO calls input() on first import unless this config exists.
RUN P=/workspace/LIBERO/libero/libero && mkdir -p /root/.libero \
    && printf "benchmark_root: $P\nbddl_files: $P/bddl_files\ninit_states: $P/init_files\ndatasets: $P/../datasets\nassets: $P/assets\n" \
       > /root/.libero/config.yaml

# The base image's entrypoint sources ROS Humble for every command, including this bash.
ENV PATH=/opt/venvs/mjlab/bin:$PATH PYTHONPATH=/workspace MUJOCO_GL=egl PYOPENGL_PLATFORM=egl NVIDIA_DRIVER_CAPABILITIES=all
WORKDIR /workspace
CMD ["bash"]
