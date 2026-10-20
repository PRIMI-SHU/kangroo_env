# Stage 1: only used to get PAL's Kangaroo MJCF out of their ROS 2 package. ROS is not in the final image.
# Base pinned by digest (osrf/ros:humble-desktop), same as Ring-Attractor's docker/ros2_kangaroo.
FROM osrf/ros@sha256:fb07245b32187d74350be25323d8ad2f8ca5c25c325759911a1eff2267a49c1e AS kangaroo
RUN apt-get update && apt-get install -y --no-install-recommends ros-humble-kangaroo-mujoco \
    && rm -rf /var/lib/apt/lists/*

# Stage 2: the environment.
FROM python:3.10-slim
# cmake + build-essential: LIBERO's egl_probe builds from source. libegl/libgl/osmesa: headless MuJoCo rendering.
RUN apt-get update && apt-get install -y --no-install-recommends \
      cmake build-essential libegl1 libgl1 libosmesa6 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*
RUN pip install --no-cache-dir "mujoco>=3.8,<3.9" "hf-libero==0.1.4" pytest

# LIBERO calls input() on first import unless ~/.libero/config.yaml exists, so write it first,
# then bake scene/object assets into the package dir (LIBERO checks there before downloading).
RUN P=$(python -c "import importlib.util; print(importlib.util.find_spec('libero').submodule_search_locations[0])")/libero \
    && mkdir -p /root/.libero \
    && printf "benchmark_root: $P\nbddl_files: $P/bddl_files\ninit_states: $P/init_files\ndatasets: $P/../datasets\nassets: $P/assets\n" \
       > /root/.libero/config.yaml \
    && python -c "from libero.libero.utils.download_utils import download_assets_from_huggingface as d; d('$P/assets')"
COPY --from=kangaroo /opt/ros/humble/share/kangaroo_mujoco/models /opt/kangaroo/models
COPY --from=kangaroo /opt/ros/humble/share/kangaroo_mujoco/package.xml /opt/kangaroo/package.xml

# Headless rendering. Uses the NVIDIA GPU under `docker run --gpus all`; set MUJOCO_GL=osmesa on hosts without EGL.
ENV MUJOCO_GL=egl PYOPENGL_PLATFORM=egl NVIDIA_DRIVER_CAPABILITIES=all

WORKDIR /app
COPY NOTICE CONTEXT.md README.md kangaroo_libero.py test_smoke.py view.py ./
CMD ["pytest", "-v"]
