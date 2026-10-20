FROM python:3.13-slim
COPY --from=ghcr.io/astral-sh/uv:0.9.5 /uv /uvx /bin/
RUN apt-get update && apt-get install -y --no-install-recommends \
      git cmake build-essential libegl1 libgl1 libosmesa6 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

COPY pal_mjlab /workspace/pal_mjlab
COPY LIBERO /workspace/LIBERO

# Env 1: pal_mjlab / mjlab 1.6.0 (Python 3.13), from PAL's lockfile. cu128 or cpu.
ARG TORCH_EXTRA=cu128
RUN UV_PROJECT_ENVIRONMENT=/opt/venvs/mjlab uv sync --frozen --project /workspace/pal_mjlab --extra ${TORCH_EXTRA} \
    && uv pip install --python /opt/venvs/mjlab pytest

# Env 2: original LIBERO (Python 3.8), its own pinned requirements. compat editable: LIBERO has no top-level __init__, so the default editable finder maps nothing.
RUN uv venv --python 3.8 /opt/venvs/libero \
    && uv pip install --python /opt/venvs/libero -r /workspace/LIBERO/requirements.txt -e /workspace/LIBERO -C editable_mode=compat

# LIBERO calls input() on first import unless this config exists.
RUN P=/workspace/LIBERO/libero/libero && mkdir -p /root/.libero \
    && printf "benchmark_root: $P\nbddl_files: $P/bddl_files\ninit_states: $P/init_files\ndatasets: $P/../datasets\nassets: $P/assets\n" \
       > /root/.libero/config.yaml

ENV PATH=/opt/venvs/mjlab/bin:$PATH PYTHONPATH=/workspace MUJOCO_GL=egl PYOPENGL_PLATFORM=egl NVIDIA_DRIVER_CAPABILITIES=all
WORKDIR /workspace
CMD ["bash"]
