# robot-learning: seated Kangaroo + LIBERO

A Docker image (ROS 2 Humble, Ubuntu 22.04, Python 3.10) for robot-learning work with PAL Robotics'
**Kangaroo** humanoid, and a base other projects build on. It ships one task-free scene: Kangaroo seated on
a chair at a table with a cup, a water bottle and an apple, for seated upper-limb rehab work. No task is
defined; you add your own.

## 1. Build and start the image

Host needs: git, curl, python3, Docker, and the NVIDIA Container Toolkit for `--gpus`.

**Fetch the sources** (once; after changing a pin in it, delete the folder that pin fills and run it again):

```bash
./setup.sh
```

The robot model, LIBERO and the object meshes are not in git. `setup.sh` clones pal_mjlab @52f5d6ed5d and
LIBERO @8f1084e313 and downloads the cup, bottle and apple, so the build can copy them into the image.

**Build** (~6 min; add `--build-arg TORCH_EXTRA=cpu` for a CPU-only torch):

```bash
docker build -t robot-learning .
```

**Start: headless** (no window, `--rm` deletes the container on exit; only files in the mounted repo stay):

```bash
mkdir -p ~/.cache/warp
docker run --rm -it --gpus all --user "$(id -u):$(id -g)" \
  -v "$PWD:/opt/robot-learning" -w /opt/robot-learning \
  -v "$HOME/.cache/warp:/cache/warp" \
  robot-learning
```

**Start: with a display** (Linux desktop, X11 or Wayland). The container is kept, so you can come back to it:

```bash
xhost +SI:localuser:$USER                                # let your user open windows from the container
docker run -it --name robot-learning-gui --gpus all --user "$(id -u):$(id -g)" \
  --device /dev/dri --group-add "$(getent group render | cut -d: -f3)" \
  -e DISPLAY=$DISPLAY -e MUJOCO_GL=glfw -e PYOPENGL_PLATFORM= \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$PWD:/opt/robot-learning" -w /opt/robot-learning \
  -v "$HOME/.cache/warp:/cache/warp" \
  robot-learning
```

Then, inside the container, run the simulation in a window (mouse to orbit and zoom; close the window to stop):

```bash
python -m kangaroo_seated.view
```

Come back to the same container later with `docker start -ai robot-learning-gui` (re-run `xhost` first after a
reboot) and run the simulation again the same way. Remove it with `docker rm robot-learning-gui`.

Both start in bash, in this repo, with ROS sourced and the mjlab venv active. The repo is mounted over
`/opt/robot-learning`, so host edits are live without a rebuild. `--user` runs as you, so files the container writes
are yours (the prompt shows `I have no name!`, which is harmless). The warp-cache mount keeps compiled GPU kernels
(the first GPU run compiles for ~4 min).

**Run the tests** (inside the container):

```bash
pytest -q tests                                          # 21 passed
```

**Record a video in one command** (headless, from the host; writes `out/kangaroo.mp4`):

```bash
docker run --rm --gpus all --user "$(id -u):$(id -g)" -v "$PWD:/opt/robot-learning" -w /opt/robot-learning \
  robot-learning python -m kangaroo_seated.view --video out/kangaroo.mp4 --seconds 5
```

- No NVIDIA GPU: drop `--gpus all` (the Warp test skips); if headless rendering fails, add `-e MUJOCO_GL=osmesa -e PYOPENGL_PLATFORM=osmesa`.

## 2. Build your own work on it

**Start in plain MuJoCo** (CPU, easiest to debug). Save as `my_experiment.py`, run `python my_experiment.py`:

```python
import mujoco
from kangaroo_seated import make_model

m, d = make_model()                                   # seated Kangaroo, chair, table, cup, bottle, apple
shoulder = m.actuator("robot/arm_right_1_joint").id   # position actuator: ctrl = target angle (rad)
for _ in range(1000):
    d.ctrl[shoulder] = -0.6                           # your controller goes here
    mujoco.mj_step(m, d)
print("right palm at", d.body("robot/hand_right_palm_link").xpos, "cup at", d.body("cup/cup").xpos)
```

**Move to mjlab for RL / many parallel envs on the GPU.** `make_scene_cfg(num_envs=N)` is the same scene as an
mjlab `SceneCfg`; put it in your env config as `cfg.scene`. The closest template is PAL's Kangaroo reaching
task, `pal_mjlab/src/pal_mjlab/tasks/reaching/`, without its locomotion terms (the legs here are locked). Use
the `mdp.reset_scene_to_default` reset event so the robot starts seated, and set `cfg.sim.nconmax = 128`,
`cfg.sim.njmax = 600` (the forearms on the table give ~55 contacts; mjlab's default sizing overflows and crashes). See [pal_mjlab's README](https://github.com/pal-robotics/pal_mjlab)
for `list-envs`, `play` and `train`.

**ROS 2:** `ros2` and `rclpy` work in every shell, so a node can read or drive the simulation; nothing is wired up yet.

**LIBERO** (separate venv): `/opt/venvs/libero/bin/python your_script.py`. To use LIBERO objects in the Kangaroo
scene, load their MuJoCo files from `LIBERO/`; never import LIBERO into the mjlab venv (numpy versions clash).

**From another project**, build on the image instead of copying this repo:

```dockerfile
FROM robot-learning
COPY . /workspace
```

Your project lives in `/workspace`, and `from kangaroo_seated import make_scene_cfg, make_model` works from anywhere.
Start it with the commands above, mounting your project at `/workspace` instead of over `/opt/robot-learning`.
`pytest /opt/robot-learning/tests` checks the base image itself.

**Promised interface** (package `kangaroo-seated` 0.1.0; any change bumps the version and gets a `vX.Y.Z` git tag):

| Name | What you get |
|---|---|
| `make_scene_cfg(num_envs=1)` | mjlab `SceneCfg` with entities `robot`, `room`, `chair`, `table`, `cup`, `bottle`, `apple` |
| `make_model()` | plain-MuJoCo `(model, data)` of the same scene, at the start pose |
| `POSE` | start joint angles by joint name (rad; `leg_*_length_joint` in m) |
| `HELD_JOINTS` | leg and trunk joints locked at `POSE` |
| `OBJECTS` | `("cup", "bottle", "apple")` |
| Names | entity-prefixed: `robot/<pal_mjlab name>`, `cup/cup`, `table/table`, …; actuators `robot/<joint name>` |

Good to know:

- Every robot joint is held by pal_mjlab's position actuators (`d.ctrl` = target angles, starting at the pose);
  legs and trunk are also locked so the robot stays seated.
- The objects stand 15 cm apart, 40 cm from the table's front edge: about 80% of the arm's reach. Masses are real:
  cup 0.35 kg, full bottle 0.5 kg, apple 68 g.
- The hands are rigid (pal_mjlab has no finger joints): objects can be pushed, not grasped.
- Reset in plain MuJoCo with `mujoco.mj_resetDataKeyframe(m, d, m.key("init_state").id)`.
- Never edit `pal_mjlab/` or `LIBERO/`: they are pinned upstream clones; `setup.sh` and a rebuild refresh them.

## What's inside

```
setup.sh              clones pinned pal_mjlab and LIBERO, downloads object assets
Dockerfile            ros:humble-ros-base + two venvs; installs everything into /opt/robot-learning
kangaroo_seated/      scene.py (the scene), view.py (viewer), seated_setup.json, apple.xml
tests/                pytest suite, including a check that another project can use the scene
```

| Venv | Contents |
|---|---|
| `/opt/venvs/mjlab` (default `python`) | pal_mjlab, mjlab 1.6.0, MuJoCo + MuJoCo Warp 3.11, `rclpy`, `kangaroo-seated` |
| `/opt/venvs/libero` | original LIBERO with its pinned requirements, MuJoCo 3.2.3 (it needs numpy 1.22, mjlab needs numpy 2) |

## Sources

- Seated setup (pose, table, chair, object layout): the lab's `seated_setup_for_libero` (not published), version 2026-10-19,
  included here as `kangaroo_seated/seated_setup.json`; `scene.py` lists every change made to it.
- Kangaroo: [pal_mjlab](https://github.com/pal-robotics/pal_mjlab) (Apache 2.0). LIBERO: [Lifelong-Robot-Learning/LIBERO](https://github.com/Lifelong-Robot-Learning/LIBERO) (MIT).
- Cup: Google Scanned Objects "Cole_Hardware_Mug_Classic_Blue" (CC BY 4.0, Google Research), via [mujoco_scanned_objects](https://github.com/kevinzakka/mujoco_scanned_objects).
- Bottle: [robosuite](https://github.com/ARISE-Initiative/robosuite) v1.5.2 (MIT).
- Apple: [YCB Object and Model Set](https://www.ycbbenchmarks.com/) "013_apple" (CC BY 4.0); B. Calli et al., IEEE Robotics & Automation Magazine, 2015.

## License

MIT for this repo's own files ([LICENSE](LICENSE)); the sources above keep their own licences.
