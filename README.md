# robot-learning: seated Kangaroo + LIBERO

A Docker image (ROS 2 Humble, Ubuntu 22.04, Python 3.10) for robot-learning work with PAL Robotics'
**Kangaroo** humanoid. It ships one task-free scene: Kangaroo seated on a chair at a table with a cup,
a water bottle and a bowl, for seated upper-limb rehab work. No task is defined; you add your own.
Terms are defined in [CONTEXT.md](CONTEXT.md).

![Seated Kangaroo at the table](docs/kangaroo_seated.png)

## 1. Build and start the image

Host needs: git, curl, python3, Docker, and the NVIDIA Container Toolkit for `--gpus`.

```bash
./setup.sh                          # 1. clone pal_mjlab @52f5d6ed5d + LIBERO @8f1084e313, download object assets
docker build -t robot-learning .    # 2. build (~6 min); add --build-arg TORCH_EXTRA=cpu for a CPU-only torch
docker run --rm -it --gpus all \
  -v "$PWD:/workspace" \
  -v "$HOME/.cache/warp:/root/.cache/warp" \
  robot-learning                    # 3. opens bash in /workspace, ROS sourced, mjlab venv active
pytest -q tests                     # 4. inside the container: check everything works (15 passed)
```

- This folder is mounted at `/workspace`: edits on the host are live in the container, no rebuild.
- The warp-cache mount keeps compiled MuJoCo Warp kernels (the first GPU run compiles for ~4 min).
- No NVIDIA GPU: drop `--gpus all` (the Warp test skips); if EGL fails, add `-e MUJOCO_GL=osmesa -e PYOPENGL_PLATFORM=osmesa`.
- Files the container writes into the mount (`out/`, `__pycache__`) are root-owned on the host.

## 2. Run the simulation

Headless video (works anywhere):

```bash
python view.py --video out/kangaroo.mp4 --seconds 5      # inside the container
```

Live window (Linux desktop; mouse to orbit/zoom), started from the host:

```bash
xhost +local:docker
docker run --rm -it --gpus all -e DISPLAY=$DISPLAY -e MUJOCO_GL=glfw -e PYOPENGL_PLATFORM= \
  -v /tmp/.X11-unix:/tmp/.X11-unix -v "$PWD:/workspace" robot-learning python view.py
```

The robot starts in the confirmed `seated_hands_on_table` pose (elbows adjusted a few degrees so both hands
rest on the table top), with every joint held by pal_mjlab's own position actuators (the same ones mjlab
uses). The forearm and hand meshes are the collision shapes, so hands touch the table and objects as drawn.

## 3. Build your own work on it

1. **Put your code in this folder** (e.g. `my_task/` next to `kangaroo_seated/`). Never edit
   `pal_mjlab/` or `LIBERO/`; they are pinned upstream clones, and `setup.sh` + a rebuild refreshes them.
2. **Start in plain MuJoCo** (CPU, easiest to debug). Save as `my_experiment.py`, run `python my_experiment.py`:

   ```python
   import mujoco
   from kangaroo_seated import make_model

   m, d = make_model()                                   # seated Kangaroo, chair, table, cup, bottle, bowl
   shoulder = m.actuator("robot/arm_right_1_joint").id   # position actuator: ctrl = target angle (rad)
   for _ in range(1000):
       d.ctrl[shoulder] = -0.6                           # your controller goes here
       mujoco.mj_step(m, d)
   print("right palm at", d.body("robot/hand_right_palm_link").xpos, "cup at", d.body("cup").xpos)
   ```

3. **Move to mjlab for RL / many parallel envs on the GPU.** `make_scene_cfg(num_envs=N)` returns the
   same scene as an mjlab `SceneCfg`; put it in your env config as `cfg.scene`. The closest template is
   PAL's Kangaroo reaching task: `pal_mjlab/src/pal_mjlab/tasks/reaching/` (`make_reaching_env_cfg()`
   plus `config/kangaroo/`, registered with `register_mjlab_task`). Drop its locomotion terms, since the
   legs here are locked. `list-envs --keyword pal` lists PAL's tasks; `play` and `train` run them (see
   [pal_mjlab's README](https://github.com/pal-robotics/pal_mjlab)).
4. **ROS 2:** `ros2` and `rclpy` work in every shell, so a node can read or drive the simulation; nothing
   is wired up yet.
5. **LIBERO** (original benchmark, separate venv): `/opt/venvs/libero/bin/python your_script.py`. To use
   LIBERO objects or layouts in the Kangaroo scene, load their MuJoCo files from `LIBERO/` into the scene
   ([ADR 0001](docs/adr/0001-one-simulator-libero-as-scene-content.md)); never import LIBERO into the mjlab venv.
6. **Add a test** under `tests/` for what you build and run `pytest -q tests`.

Good to know:

- Plain MuJoCo and mjlab use the same robot: pal_mjlab's position actuators on arms, legs and trunk
  (`d.ctrl` = target angles, initialised to the pose), and equality locks keeping legs and trunk seated.
- Plain MuJoCo body names are `robot/...`, `cup`, `bottle`, `bowl`, `chair`, `table`.
- mjlab: Body
  names are entity-prefixed (`cup/cup`). Use the `mdp.reset_scene_to_default` reset event so the robot
  starts seated; a bare `sim.reset()` returns to qpos0.
- The hands are rigid (pal_mjlab comments out every finger joint): objects can be pushed or scooped, not grasped.
- The seated pose is within Kangaroo's real joint limits (checked by the tests).

## What's inside

```
setup.sh              clones pinned pal_mjlab and LIBERO, downloads object assets
Dockerfile            ros:humble-ros-base + two venvs
kangaroo_seated/      the scene: scene.py (plain MuJoCo), mjlab_cfg.py (mjlab), seated_setup.json
view.py               live viewer or headless video
tests/                pytest suite
pal_mjlab/ LIBERO/ assets/   fetched by setup.sh (git-ignored)
```

| Venv | Python | Contents |
|---|---|---|
| `/opt/venvs/mjlab` (default `python`) | 3.10 | pal_mjlab, mjlab 1.6.0, MuJoCo + MuJoCo Warp 3.11 (PAL's lockfile), `rclpy` |
| `/opt/venvs/libero` | 3.10 | original LIBERO with its pinned requirements, MuJoCo 3.2.3 |

The venvs stay separate because LIBERO pins numpy 1.22 and mjlab needs numpy 2.

## Sources

- Seated setup (base pose, joint angles, table, chair, object layout): `seated_setup_for_libero`, `seated_setup.json` version 2026-10-19.
- Kangaroo: [pal_mjlab](https://github.com/pal-robotics/pal_mjlab). LIBERO: [Lifelong-Robot-Learning/LIBERO](https://github.com/Lifelong-Robot-Learning/LIBERO).
- Objects: cup and bowl from Google Scanned Objects via [mujoco_scanned_objects](https://github.com/kevinzakka/mujoco_scanned_objects); bottle from robosuite v1.5.2. See [NOTICE](NOTICE).
