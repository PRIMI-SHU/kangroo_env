# robot-learning (pal_mjlab + LIBERO, seated Kangaroo scene) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rebuild this folder as `robot-learning/`: a bash-first Docker image with two envs (pal_mjlab/mjlab 1.6.0 and original LIBERO), and one fixed, task-free scene where Kangaroo sits (confirmed seated setup) at a table with a cup, a water bottle and a bowl in reach.

**Architecture:** `setup.sh` clones both official repos at pinned commits into this folder. The Dockerfile copies them to `/workspace/...` at build time and installs them editable there, so at runtime the `-v "$PWD:/workspace"` mount puts the live clones exactly where the editable installs point. Our package `kangaroo_seated/` (mjlab env) builds the scene as an `MjSpec` from pal_mjlab's Kangaroo XML, the confirmed `seated_setup.json` (base pose, joint pose, chair and table boxes), LIBERO's room XML and downloaded object models, and wraps it in an mjlab `SceneCfg`.

**Tech Stack:** Docker, `python:3.13-slim`, uv, mjlab 1.6.0, MuJoCo / MuJoCo Warp 3.11.x, pal_mjlab @ `52f5d6ed5d`, LIBERO @ `8f1084e313` (Python 3.8, robosuite 1.4.0), pytest.

**Spec:** `docs/superpowers/specs/2026-10-19-robot-learning-docker-design.md`

## Global Constraints

- pal_mjlab pinned to `52f5d6ed5d`; LIBERO pinned to `8f1084e313`; installed from official GitHub only.
- mjlab env: Python 3.13, `uv sync --frozen` (lock gives mjlab 1.6.0, mujoco 3.11.0, mujoco-warp 3.11.0, torch 2.10.0). No MuJoCo other than 3.11.x.
- LIBERO env: Python 3.8, LIBERO `requirements.txt`, separate venv. Never mix with mjlab deps.
- Envs in `/opt/venvs/mjlab` and `/opt/venvs/libero`; `WORKDIR /workspace`; `CMD ["bash"]`; `MUJOCO_GL=egl`.
- Build arg `TORCH_EXTRA=cu128` (default) | `cpu`.
- No ROS, no `hf-libero`, no LIBERO asset download; `~/.libero/config.yaml` pre-written (LIBERO's import calls `input()` otherwise).
- Never modify files inside `pal_mjlab/` or `LIBERO/`.
- Seating copied, not re-derived: `kangaroo_seated/seated_setup.json` = copy of `../seated_setup_for_libero/seated_setup.json` (version 2026-10-19). Base `(-0.3544, 0, 0.7479)` quat `(1,0,0,0)`; pose `seated_arms_at_side`; `chair.boxes`; `table.boxes` (top 0.82). pal_mjlab convention: no arm_5 negation.
- Objects: cup = GSO `Cole_Hardware_Mug_Classic_Blue`, bowl = GSO `Threshold_Bead_Cereal_Bowl_White` (from `kevinzakka/mujoco_scanned_objects` @ `6ff8d275ce`), bottle = robosuite `v1.5.2` `bottle.xml` + `meshes/bottle.stl` + `textures/glass.png`. Downloaded by `setup.sh` into `assets/`.
- Scene is fixed and task-free: no LIBERO task ids, no randomisation, no rewards.
- This folder is not a git repo: no commit steps.

## Review Focus

1. Mounting a folder where `setup.sh` was never run (no clones): clear error naming `./setup.sh`, not a deep `ImportError`. → Task 1 `test_clones_present`.
2. Running without `--gpus all`: everything passes except the Warp check, which skips. → Task 4 skip marker.
3. Objects resting on the table at t=0 must not explode or fall through. → Task 2 `test_objects_stay_on_table`.
4. Host edits in `pal_mjlab/` take effect without rebuild (editable path = mount path). → Task 1 `test_editable_paths_point_at_workspace`.
5. Headless (EGL) render produces a non-black frame. → Task 2 `test_arm_moves_and_renders`.

---

### Task 1: Clones, Docker image, env smoke tests

**Files:**
- Create: `setup.sh`, `tests/test_envs.py`; replace `Dockerfile`, `.dockerignore`; copy `../seated_setup_for_libero/seated_setup.json` → `kangaroo_seated/seated_setup.json`
- Delete: `kangaroo_libero.py`, `view.py`, `test_smoke.py` (replaced by Tasks 2–5)

**Interfaces:**
- Produces: image `robot-learning`; `/opt/venvs/mjlab/bin/python` (on `PATH`), `/opt/venvs/libero/bin/python`; tests run as `pytest -v tests`.

- [ ] **Step 1: Write `tests/test_envs.py`**

```python
import pathlib, subprocess

def test_clones_present():
    for d in ("pal_mjlab", "LIBERO"):
        assert pathlib.Path("/workspace", d).is_dir(), f"missing {d}/: run ./setup.sh on the host"

def test_mjlab_stack_versions():
    import mujoco, importlib.metadata as md
    assert mujoco.__version__.startswith("3.11.")
    assert md.version("mujoco-warp").startswith("3.11.")
    assert md.version("mjlab") == "1.6.0"
    from pal_mjlab.robots.pal_kangaroo.kangaroo_constants import KANGAROO_HANDS_XML
    assert KANGAROO_HANDS_XML.exists()

def test_editable_paths_point_at_workspace():
    import pal_mjlab
    assert pal_mjlab.__file__.startswith("/workspace/pal_mjlab/")
    out = subprocess.run(["/opt/venvs/libero/bin/python", "-c", "import libero.libero as l; print(l.__file__)"],
                         capture_output=True, text=True, stdin=subprocess.DEVNULL, timeout=120)
    assert out.stdout.startswith("/workspace/LIBERO/"), out.stderr

def test_libero_imports_without_prompt_and_resets():
    # stdin=DEVNULL: an input() prompt raises EOFError and fails the test.
    code = ("import os; from libero.libero import benchmark, get_libero_path;"
            "from libero.libero.envs import OffScreenRenderEnv;"
            "t=benchmark.get_benchmark_dict()['libero_spatial']().get_task(0);"
            "e=OffScreenRenderEnv(bddl_file_name=os.path.join(get_libero_path('bddl_files'),t.problem_folder,t.bddl_file),"
            "camera_heights=64,camera_widths=64); e.reset(); e.close(); print('OK')")
    out = subprocess.run(["/opt/venvs/libero/bin/python", "-c", code], capture_output=True, text=True,
                         stdin=subprocess.DEVNULL, timeout=600)
    assert "OK" in out.stdout, out.stderr[-2000:]
```

- [ ] **Step 2: `setup.sh`**: idempotent; for `pal_mjlab https://github.com/pal-robotics/pal_mjlab.git 52f5d6ed5d` and `LIBERO https://github.com/Lifelong-Robot-Learning/LIBERO.git 8f1084e313`: clone if missing, then `git -C <dir> checkout <sha>`. Then download (skip files already present) into `assets/cup/` and `assets/bowl/` every file of `models/<GSO name>/` at `6ff8d275ce` (`model.xml`, `model.obj`, `model_collision_*.obj`, `texture.png`; list via the GitHub tree API or a sparse git checkout), and into `assets/bottle/` robosuite `v1.5.2` `robosuite/models/assets/objects/bottle.xml`, `.../objects/meshes/bottle.stl`, `.../textures/glass.png` keeping their relative layout so `bottle.xml` resolves. Run `./setup.sh` → `pal_mjlab/`, `LIBERO/`, `assets/{cup,bowl,bottle}/` exist. Add test `test_assets_present` in `tests/test_envs.py` asserting the three XML files exist (message: run ./setup.sh).

- [ ] **Step 3: `Dockerfile`**
  - `FROM python:3.13-slim`; `COPY --from=ghcr.io/astral-sh/uv:<pinned release tag> /uv /uvx /bin/`.
  - apt: `git libegl1 libgl1 libosmesa6 libglib2.0-0` (add `cmake build-essential` only if the LIBERO install fails without them).
  - `COPY pal_mjlab /workspace/pal_mjlab` and `COPY LIBERO /workspace/LIBERO`.
  - mjlab env: `UV_PROJECT_ENVIRONMENT=/opt/venvs/mjlab uv sync --frozen --project /workspace/pal_mjlab --extra ${TORCH_EXTRA}`; then `uv pip install --python /opt/venvs/mjlab pytest`.
  - LIBERO env: `uv venv --python 3.8 /opt/venvs/libero`; `uv pip install --python /opt/venvs/libero -r /workspace/LIBERO/requirements.txt -e /workspace/LIBERO`; add `torch` (CPU wheel) only if `import libero.libero.envs` needs it.
  - LIBERO config: one `printf` writing `/root/.libero/config.yaml` with keys `benchmark_root, bddl_files, init_states, datasets, assets` under `/workspace/LIBERO/libero/libero` (LIBERO's `get_default_path_dict`).
  - `ENV PATH=/opt/venvs/mjlab/bin:$PATH PYTHONPATH=/workspace MUJOCO_GL=egl PYOPENGL_PLATFORM=egl NVIDIA_DRIVER_CAPABILITIES=all`; `WORKDIR /workspace`; `CMD ["bash"]`.
  - `.dockerignore`: `out/`, `**/.venv`, `**/__pycache__`, `docs/`.

- [ ] **Step 4: Build and test**

Run: `docker build -t robot-learning . && docker run --rm -v "$PWD:/workspace" robot-learning pytest -v tests/test_envs.py`
Expected: 5 passed. `docker run --rm -it -v "$PWD:/workspace" robot-learning` lands in bash at `/workspace`.

---

### Task 2: Seated scene as MjSpec (`kangaroo_seated/scene.py`)

**Files:**
- Create: `kangaroo_seated/__init__.py`, `kangaroo_seated/scene.py`, `tests/test_scene.py`

**Interfaces:**
- Consumes: `KANGAROO_HANDS_XML` (pal_mjlab); LIBERO room at `/workspace/LIBERO/libero/libero/assets/scenes/libero_tabletop_base_style.xml`; `/workspace/assets/{cup,bowl,bottle}/`.
- Produces (re-exported by `kangaroo_seated/__init__.py`):
  - `SETUP: dict` (parsed `seated_setup.json`); `POSE: dict[str, float]` = `SETUP["poses"]["seated_arms_at_side"]["joints"]`
  - `HELD_JOINTS: list[str]`: `leg_*` and `pelvis_*` names in `POSE` except passive `leg_*_femur_joint`/`leg_*_knee_joint` (14 entries); arms not held.
  - `OBJECTS = ("cup", "bottle", "bowl")`
  - `build_spec() -> mujoco.MjSpec`: full scene; robot attached with prefix `"robot/"`; bodies `cup`, `bottle`, `bowl` each with a free joint; chair/table geoms with the JSON's box names.
  - `make_model() -> tuple[mujoco.MjModel, mujoco.MjData]`: compiled, `POSE` set (femur/knee come solved from the JSON: no settle pass), `mj_forward` done.

- [ ] **Step 1: Write `tests/test_scene.py`**

```python
import os, imageio, numpy as np, mujoco
from kangaroo_seated import POSE, HELD_JOINTS, OBJECTS, make_model

def step_1s(m, d): mujoco.mj_step(m, d, nstep=int(1.0 / m.opt.timestep))

def test_matches_confirmed_setup():
    m, d = make_model()
    assert np.allclose(d.body("robot/base_link").xpos, [-0.3544, 0, 0.7479], atol=1e-4)
    for s in ("left", "right"):
        assert abs(d.body(f"robot/{s}_sole_link").xpos[2]) < 0.005  # soles on the floor (setup: -0.5 mm)
    for n, q in POSE.items():
        lo, hi = m.joint(f"robot/{n}").range
        assert lo - 1e-9 <= q <= hi + 1e-9, n

def test_seated_scene_holds():
    m, d = make_model(); step_1s(m, d)
    assert np.isfinite(d.qpos).all()
    assert max(abs(d.joint(f"robot/{n}").qpos[0] - POSE[n]) for n in HELD_JOINTS) < 0.05
    assert any("chair" in (m.body(m.geom_bodyid[c.geom1]).name, m.body(m.geom_bodyid[c.geom2]).name)
               for c in d.contact[:d.ncon])

def test_objects_stay_on_table():
    m, d = make_model()
    z0 = {o: d.body(o).xpos[2] for o in OBJECTS}
    step_1s(m, d)
    for o in OBJECTS:
        assert abs(d.body(o).xpos[2] - z0[o]) < 0.02, o
        assert np.linalg.norm(d.body(o).cvel) < 0.05, o

def test_arm_moves_and_renders():
    m, d = make_model()
    j = m.joint("robot/arm_left_1_joint"); q0 = d.qpos[j.qposadr[0]]
    a = next(i for i in range(m.nu) if m.actuator_trnid[i, 0] == j.id)
    d.ctrl[a] = q0 + 0.5  # position target (pal_mjlab uses position actuators); a torque also moves it
    mujoco.mj_step(m, d, nstep=500)
    assert abs(d.qpos[j.qposadr[0]] - q0) > 0.05
    cam = mujoco.MjvCamera(); cam.lookat[:] = [-0.1, 0, 0.75]  # setup's preview camera
    cam.distance, cam.azimuth, cam.elevation = 3.2, 90, -10
    with mujoco.Renderer(m, 480, 640) as r:
        r.update_scene(d, camera=cam); img = r.render()
    assert img.max() > 0
    os.makedirs("out", exist_ok=True); imageio.imwrite("out/kangaroo_seated.png", img)
```

- [ ] **Step 2: Run → FAIL** (`ModuleNotFoundError: kangaroo_seated`).
Run: `docker run --rm -v "$PWD:/workspace" robot-learning pytest -v tests/test_scene.py`

- [ ] **Step 3: Implement `kangaroo_seated/scene.py`**
  - Room: `MjSpec.from_file(<LIBERO assets>/scenes/libero_tabletop_base_style.xml)`; delete LIBERO's table body/geoms (keep floor, walls, lights). If MuJoCo 3.11 rejects an attribute, patch via `MjSpec`, never the file. Floor must be at z = 0 (setup frame).
  - Robot: `MjSpec.from_file(KANGAROO_HANDS_XML)`; delete free joint `origin` and keyframes; attach at a frame with the setup's base pos/quat, prefix `"robot/"`.
  - Chair and table: one static geom per entry of `SETUP["chair"]["boxes"] + SETUP["table"]["boxes"]` (box, `size_half`, `pos`, `rgba`, same names), as in `../seated_setup_for_libero/apply_seated_setup.py`.
  - Hold: one `mjEQ_JOINT` equality per `HELD_JOINTS` entry, `data=[POSE[n]]+[0]*10`, `solref=[2*timestep, 1]`. Do not hold femur/knee (closed chain does) or arms.
  - Objects: attach `assets/cup/model.xml`, `assets/bowl/model.xml`, `assets/bottle/bottle.xml` (`bottle.xml`'s root has `body > body name="object"`: wrap so the free joint sits on the moving body), each under a new body named `cup`/`bowl`/`bottle` with a free joint; set object `qpos` so each rests on the table top (z from its compiled collision AABB), on the table centre line region nearest the robot, ~0.15 m apart in y, x from constant `OBJECT_X` (start `-0.05`; Task 3 tunes). GSO geoms: visual group 2, collision group 3 as shipped.

- [ ] **Step 4: Run → 4 passed**; open `out/kangaroo_seated.png`: seated robot on the chair at the table, cup, bottle and bowl on the table.

---

### Task 3: Reach: place the objects so all three are reachable

**Files:**
- Modify: `kangaroo_seated/scene.py` (object placement constants only; robot/chair/table are fixed by the setup); `tests/test_scene.py`

**Interfaces:**
- Consumes: `make_model()`, `OBJECTS`. Produces: final constants; no API change.

- [ ] **Step 1: Add the failing test**

```python
def test_each_object_reachable():
    m, d = make_model()
    for o in OBJECTS:
        best = min(min_palm_distance(m, d, s, d.body(o).xpos.copy()) for s in ("left", "right"))
        assert best < 0.10, (o, best)
```

  Test helper `min_palm_distance(m, d, side: str, target: np.ndarray) -> float`: damped-least-squares IK on the 5 `robot/arm_<side>_*_joint`s using `mj_jacBody` on `robot/hand_<side>_palm_link`, clamped to joint ranges, ≤ 200 iterations, kinematics only (`mj_kinematics` on a copy of `d`); returns the final palm–target distance.

- [ ] **Step 2: Run → record distances** (expect FAIL if out of reach).
- [ ] **Step 3: Move `OBJECT_X` (and the y spacing) toward the robot until it passes.** Reach wins over exact centring. If even the table front edge is out of reach, stop and report (do not move the robot: the setup is confirmed).
- [ ] **Step 4: Run `tests/test_scene.py` → all pass**; re-check the render.

---

### Task 4: mjlab `SceneCfg` + MuJoCo Warp step

**Files:**
- Create: `kangaroo_seated/mjlab_cfg.py`, `tests/test_mjlab_cfg.py`; modify `kangaroo_seated/__init__.py` (re-export)

**Interfaces:**
- Consumes: `scene.py` (split `build_spec()` into `room_spec()`, `robot_spec()`, `furniture_spec()`, `object_spec(name)` if entities need separate specs; `build_spec()` keeps working).
- Produces: `make_scene_cfg(num_envs: int = 1) -> mjlab.scene.SceneCfg`, entities `robot`, `chair`, `table`, `room`, `cup`, `bottle`, `bowl`. Robot: fixed-base `EntityCfg`, `init_state` = setup base pose + `POSE`, legs/trunk held (builder's call: equality locks via `SceneCfg.spec_fn`, or position targets).

- [ ] **Step 1: Write tests**

```python
import pytest, torch
from kangaroo_seated import make_scene_cfg, OBJECTS

def test_scene_cfg_entities():
    assert set(make_scene_cfg().entities) >= {"robot", "chair", "table", "room", *OBJECTS}

@pytest.mark.skipif(not torch.cuda.is_available(), reason="no GPU: MuJoCo Warp check skipped")
def test_scene_steps_on_warp():
    # Build mjlab's Scene + Simulation from make_scene_cfg(num_envs=4) on "cuda:0" (mjlab 1.6.0 APIs),
    # step 100 times, assert robot joint positions are finite and held joints within 0.05 rad of POSE.
    ...
```

- [ ] **Step 2: Run → FAIL.** **Step 3: Implement.** If Warp rejects LIBERO mesh collisions, swap object/arena *collision* geoms to convex hulls or primitives (visuals unchanged) and say so in the README (spec allows this). **Step 4: Run → PASS** with `--gpus all`; SKIP without.

---

### Task 5: Viewer, docs, cleanup

**Files:**
- Create: `view.py`; modify `README.md`, `CONTEXT.md`, `NOTICE`

**Interfaces:**
- Consumes: `make_model()`.

- [ ] **Step 1: `view.py`**: no task/suite arguments (user: "just a working simulation"). `python view.py` opens `mujoco.viewer`; `python view.py --video out/kangaroo.mp4 --seconds 5` renders headless with the test's camera. Only args: `--video`, `--seconds`.
- [ ] **Step 2: Verify** `docker run --rm --gpus all -v "$PWD:/workspace" robot-learning python view.py --video out/kangaroo.mp4 --seconds 2` → non-empty mp4.
- [ ] **Step 3: Docs.** README: layout tree; `./setup.sh` → `docker build -t robot-learning .` (`--build-arg TORCH_EXTRA=cpu` option) → `docker run --rm -it --gpus all -v "$PWD:/workspace" robot-learning`; tests, viewer (X11 command adapted to `/workspace`), LIBERO env usage (`/opt/venvs/libero/bin/python`), scene API (`make_model`, `make_scene_cfg`), version table, seated pose within real limits. CONTEXT.md: replace *Combined scene*/*Welded base* with **Seated scene**, **pal_mjlab**, **Fixed base**. NOTICE: pal_mjlab, LIBERO, robosuite (MIT, bottle asset) and Google Scanned Objects (CC-BY 4.0: name both models) attributions; drop the ROS package. README credits `seated_setup_for_libero` as the source of the seated setup.
- [ ] **Step 4: Full run** `docker run --rm --gpus all -v "$PWD:/workspace" robot-learning pytest -v tests` → all pass.
