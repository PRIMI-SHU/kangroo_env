# Kangaroo + LIBERO + MuJoCo environment

A Docker image that puts PAL Robotics' **Kangaroo** humanoid into **LIBERO** manipulation scenes,
simulated with **MuJoCo**, ready for the team to plug RL algorithms into.
Scope: the environment only. Terms used here are defined in [CONTEXT.md](CONTEXT.md).

![Kangaroo at the LIBERO table](out/kangaroo_libero.png)

## The pieces

| Piece | What it is | Where it comes from |
|---|---|---|
| MuJoCo 3.8.x | Physics engine: simulates joints, contacts, cameras from MJCF (XML) models | PyPI `mujoco` |
| LIBERO | Benchmark of 130 language-specified manipulation tasks (scenes + BDDL goals + demos), built on robosuite | PyPI `hf-libero==0.1.4` (pins robosuite 1.4.0, mujoco < 3.9) |
| Kangaroo | PAL's biped humanoid; default variant has 5-DoF arms and RH8D hands (38 actuators) | MJCF copied from PAL's ROS 2 package `ros-humble-kangaroo-mujoco` at build time |

ROS is used **only** in a throwaway build stage to obtain PAL's model files; the final image is
plain Python, which keeps RL stepping fast and resets simple.

## Build and run

```bash
docker build -t kangaroo-libero .
docker run --rm --gpus all -v "$PWD/out:/app/out" kangaroo-libero      # runs the smoke test
docker run --rm -it --gpus all kangaroo-libero bash                     # interactive shell
```

No NVIDIA GPU: drop `--gpus all` (EGL falls back to software). If EGL is unavailable, add
`-e MUJOCO_GL=osmesa -e PYOPENGL_PLATFORM=osmesa`. All three modes pass the smoke test.

The smoke test (`test_smoke.py`) checks, in order:
1. pinned versions (MuJoCo 3.8.x, robosuite 1.4.0) and LIBERO imports without its interactive setup prompt;
2. a stock LIBERO task (with its Franka Panda) resets, steps and renders headless;
3. PAL's Kangaroo loads and steps (38 actuators);
4. the **combined scene**: LIBERO task arena + objects, Panda removed, Kangaroo attached; it steps,
   the welded base holds, and a frame is written to `out/kangaroo_libero.png`;
5. the **seated scene**: Kangaroo on a chair at the table, legs locked within real joint limits,
   arms still driveable; a frame is written to `out/kangaroo_seated.png`.

## Watching the simulation

**Live 3D window** (Linux desktop; mouse to orbit/zoom, Space pause, Backspace reset):

```bash
xhost +local:docker          # once per login: let containers open windows on your screen
docker run --rm -it --gpus all -e DISPLAY=$DISPLAY -e MUJOCO_GL=glfw -e PYOPENGL_PLATFORM= \
  -v /tmp/.X11-unix:/tmp/.X11-unix kangaroo-libero python view.py
```

**Video, no screen needed** (servers, clusters, SSH):

```bash
docker run --rm --gpus all -v "$PWD/out:/app/out" kangaroo-libero \
  python view.py --video out/kangaroo.mp4 --seconds 5 --camera frontview
```

Options: `--seated` puts Kangaroo on a chair; `--suite libero_object --task 3` picks another LIBERO task; cameras are `frontview`,
`agentview`, `sideview`, `birdview`, `galleryview`. With no controller, Kangaroo's 24 torque
motors output zero, so the arms settle under gravity; driving them is the RL policy's job.
`MESA ... iris` warnings in the live window are harmless (software OpenGL fallback).

## Using it

```python
import mujoco
from kangaroo_libero import make_scene

m, d, language = make_scene("libero_spatial", task_id=0)   # any LIBERO suite / task index
print(language)                  # "pick up the black bowl between the plate and the ramekin ..."
d.ctrl[:] = 0.0                  # 38 Kangaroo actuators: m.actuator(i).name
mujoco.mj_step(m, d)
```

`make_scene(..., seated=True)` sits Kangaroo on a box chair for seated upper-limb (rehab) work:
the pelvis is welded lower and the leg joints are held by equality constraints in a pose within
PAL's real joint limits (`SEATED_LEGS`), so only arm and hand actuators have any effect.

![Kangaroo seated at the LIBERO table](out/kangaroo_seated.png)

Kangaroo's names are prefixed `kangaroo/`; LIBERO objects keep their names (e.g. `plate_1_joint0`).
Cameras from LIBERO remain: `frontview`, `agentview`, `sideview`, `birdview`, `galleryview`.

## What this is, and what it is not (yet)

- **Is:** a reproducible environment where Kangaroo, LIBERO scenes and MuJoCo run together, headless, on CPU or GPU.
- **Is not:** a LIBERO task for Kangaroo. LIBERO's goal checking (BDDL predicates), rewards and
  Gym-style `reset/step` are tied to robosuite's robot classes and are **not** wired to Kangaroo.
  The next step for the team is either a thin Gymnasium wrapper around `make_scene` (reward from
  object poses) or registering Kangaroo as a robosuite robot so LIBERO's own task logic applies.
- **LIBERO demos are not included.** They were recorded on a Panda arm, so they cannot drive Kangaroo without retargeting.
- **Base is welded** at PAL's home pelvis height (0.9 m), separating manipulation from balance.
  For balance/locomotion, keep the free joint (see the comment in `make_scene`).
- The image is ~13 GB, mostly the CUDA build of PyTorch that LIBERO depends on (and that RL training will want anyway).
