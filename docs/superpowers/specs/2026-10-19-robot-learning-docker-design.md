# robot-learning: Kangaroo (pal_mjlab) + LIBERO scenes, Docker rebuild

Date: 2026-10-19, revised 2026-10-20. Status: **approved** (revision approved 2026-10-20).

> **Revision 2026-10-20 (user-approved, supersedes the Docker base and Python versions below):**
> base image `ros:humble-ros-base-jammy` (ROS 2 Humble, Ubuntu 22.04); both venvs use its system
> **Python 3.10** (mjlab venv from PAL's lockfile, which supports 3.10; LIBERO venv moves from 3.8
> to 3.10 with `mujoco==3.2.3` pinned). ROS is sourced in every shell, not wired to the simulator.
> LIBERO stays a separate venv and contributes scene content as MuJoCo files: see
> `docs/adr/0001-one-simulator-libero-as-scene-content.md`. Terms: `CONTEXT.md`.
>
> **Revision 2026-10-20 (user-requested):** the bowl is replaced by an apple (YCB `013_apple`, CC BY 4.0;
> no apple exists in Google Scanned Objects or robosuite); objects move to 40 cm from the table's front edge
> (`OBJECT_X = 0.10`, ~80% of measured reach). Start pose is `seated_hands_on_table` with small elbow/shoulder
> changes so both hands rest on the table; arms use pal_mjlab's actuators; forearm/hand meshes collide.
>
> **Revision 2026-10-20 (architecture review, user-approved):** the image is also a base for other projects
> (ADR 0002). Everything it provides lives in `/opt/robot-learning`; `/workspace` is for the consuming project, and
> developers mount this repo over `/opt/robot-learning`. `kangaroo_seated` is an editable-installed package
> (`kangaroo-seated` 0.1.0, no `PYTHONPATH`); `make_model()` compiles the mjlab `SceneCfg` (one scene assembly,
> mjlab names such as `cup/cup`); the viewer is `python -m kangaroo_seated.view`. Promised interface: README section 4.

## Intent and why

The project gives the team a ready simulation environment for **seated upper-limb rehabilitation**
work with PAL's Kangaroo: Kangaroo sits on a chair at a LIBERO table and works with its arms.
The team writes its own tasks, rewards and RL on top. This project supplies the **scene only**.

The current build (ROS stage to extract the Kangaroo MJCF, `hf-libero` from PyPI, MuJoCo 3.8, one
Python env) is being replaced because the user wants only **current, official** upstream code:

- Kangaroo comes from **pal_mjlab** (PAL's official mjlab integration), not the ROS package.
- Simulation runs on **mjlab 1.6.0 + MuJoCo / MuJoCo Warp 3.11.x** (GPU-capable).
- LIBERO comes from its **official GitHub repo**, in its own legacy-compatible environment.

## Verified facts (official sources, checked 2026-10-19)

| Item | Fact | Source |
|---|---|---|
| pal_mjlab | `requires-python >=3.10,<3.14`; `.python-version` = **3.13**; depends on `mjlab>=1.6.0`; installed with `uv sync`; extras `cu128`/`cpu`; last commit 2026-10-14 (`52f5d6ed5d`) | github.com/pal-robotics/pal_mjlab |
| pal_mjlab Kangaroo | Ships its own MJCF: `src/pal_mjlab/robots/pal_kangaroo/xmls/kangaroo_hands.xml` (5-DoF arms + hands). Joint names and limits match the ROS `kangaroo_mujoco` model (`leg_*_length_joint`, passive `femur`/`knee`, `leg_*_closed_kin_joint` connect with `solref 0.004`) | same |
| mjlab | latest **1.6.0**; pins `mujoco~=3.11.0`, `mujoco-warp~=3.11.0`, `torch>=2.7`, `warp-lang>=1.14` | pypi.org/pypi/mjlab |
| MuJoCo latest | 3.16.0: **must not be used** (mjlab pins 3.11) | pypi |
| mjlab API | `EntityCfg(spec_fn=..., init_state=..., articulation=...)`; an entity without a free joint is **fixed-base**; `SceneCfg(terrain, entities, spec_fn)`; plugins register via entry point `mjlab.tasks` | mjlab v1.6.0 source |
| LIBERO | Python **3.8.13**, `robosuite==1.4.0`, `bddl==1.0.1`, `numpy==1.22.4`, torch 1.11+cu113 documented; last commit 2025-03-15 (`8f1084e313`); assets are in the repo | github.com/Lifelong-Robot-Learning/LIBERO |
| LIBERO first import | `libero/libero/__init__.py` calls `input()` if `~/.libero/config.yaml` is missing, so the config must be pre-written. Asset download is **not** needed with a git clone | same |

## Decisions (user's)

- Two separate environments, side by side; **this folder becomes the `robot-learning/` root**.
- Only current official code: no `hf-libero`, no ROS.
- Docker image **starts in bash** with the **current directory mounted**.
- Simulation is mjlab-based; LIBERO supplies the table room and object assets.
- **One fixed scene** for the whole simulation: a chair, seated Kangaroo, and **three objects
  grouped at the centre of the table, all within the robot's reach: a cup, a water bottle and a
  bowl**. Realistic downloaded models preferred over hand-made shapes.
- **No task**: no LIBERO task id, goal or success check; the script only loads the scene and runs
  physics ("just a working simulation").
- **Seating, chair and table are copied from the confirmed setup** in
  `../seated_setup_for_libero/seated_setup.json` (version 2026-10-19), not re-derived.
- Seated pose respects Kangaroo's **real joint limits** (from the previous round).
- Deliverable is **a scene**, not an mjlab task: no rewards, terminations or RL configs.

## Target layout

```
robot-learning/                    # this folder, mounted at /workspace
├── Dockerfile
├── setup.sh                       # clones both repos at the pinned commits (idempotent)
├── pal_mjlab/                     # official clone @ 52f5d6ed5d
├── LIBERO/                        # official clone @ 8f1084e313
├── kangaroo_seated/               # our package (mjlab env): seated Kangaroo + chair + scene
├── test_smoke.py
├── README.md, CONTEXT.md, NOTICE
└── docs/superpowers/specs/        # this doc
```

Retired: `kangaroo_libero.py`, `view.py` (replaced by `kangaroo_seated/` and its viewer), the ROS
build stage, `hf-libero`, MuJoCo 3.8, the asset-download step.

## Docker (my calls unless marked)

- Base: official `python:3.13-slim`. `uv` from the official `ghcr.io/astral-sh/uv` image
  (`COPY --from=`, pinned release tag).
- System libs for headless rendering: `git`, `libegl1`, `libgl1`, `libosmesa6`, `libglib2.0-0`
  (+ `cmake`/`build-essential` only if a LIBERO dep must compile).
- Flow: user runs `./setup.sh` once on the host (clones into this folder), then `docker build`.
  The build context includes the clones.
- **Dependencies are baked into the image, outside the mount:**
  - `/opt/venvs/mjlab` (Python 3.13): pal_mjlab's locked deps (`uv sync --frozen --extra cu128`
    with `UV_PROJECT_ENVIRONMENT`), plus `kangaroo_seated`.
  - `/opt/venvs/libero` (Python 3.8 from `uv python install 3.8`): LIBERO's `requirements.txt`.
- **Source is live from the mount:** both packages are installed so that imports resolve to
  `/workspace/pal_mjlab`, `/workspace/LIBERO` and `/workspace/kangaroo_seated` (editable installs
  or `.pth` files). Editing the clones on the host takes effect without rebuilding.
- LIBERO config: a single line writing `~/.libero/config.yaml` with paths under `/workspace/LIBERO`.
- `ENV MUJOCO_GL=egl`, `WORKDIR /workspace`, `CMD ["bash"]`.
- Run: `docker run --rm -it --gpus all -v "$PWD:/workspace" robot-learning` (user's).

## Scene contents

- **Seated setup (user's, confirmed):** copy `seated_setup.json` into `kangaroo_seated/` and read
  it at runtime. Values are in pal_mjlab `kangaroo_hands.xml` convention (no arm_5 negation).
  - Base `base_link` fixed at `(-0.3544, 0, 0.7479)`, quat `(1, 0, 0, 0)`.
  - Start pose `seated_arms_at_side` (all 28 joints incl. solved femur/knee); hand joints left at
    the model default.
  - Chair: the JSON's `chair.boxes` (seat top 0.40 m, backrest touching the back handle).
  - Table: the JSON's `table.boxes` (top 0.82 m, front edge x = -0.304, 0.9 x 1.5 m).
  - Expected checks (from the setup's own output): soles at ~0 mm, closed-chain residual ~1e-7 m,
    femur shells touching the seat front edge (~4 mm), table clear of the robot.
- **Room (my call):** floor and walls from LIBERO's `scenes/libero_tabletop_base_style.xml` in the
  `LIBERO/` clone, with LIBERO's own table removed (our table replaces it). Loaded directly under
  MuJoCo 3.11; patched via `MjSpec` if needed, never by editing the clone.
- **Objects** (loose, free joint each, resting on the table top, grouped at the table centre line
  nearest the robot, ~0.15 m apart, within reach; reach wins over exact centring):
  - **cup**: Google Scanned Objects `Cole_Hardware_Mug_Classic_Blue` (textured scan, handle);
  - **bowl**: Google Scanned Objects `Threshold_Bead_Cereal_Bowl_White` (textured scan);
  - both from `github.com/kevinzakka/mujoco_scanned_objects` @ `6ff8d275ce` (repo MIT, models
    CC-BY 4.0, attribution in NOTICE); each model is `model.xml` + visual `model.obj` +
    convex-decomposed `model_collision_*.obj` + `texture.png`;
  - **water bottle**: robosuite **v1.5.2** (latest release, MIT) `models/assets/objects/bottle.xml`
    + `meshes/bottle.stl` + `textures/glass.png` (bottle-shaped, ~16 cm, untextured glass; no
    scanned water bottle exists in the collection).
  - Downloaded by `setup.sh` into `assets/` (pinned URLs), not committed by hand, not hand-built.

The `LIBERO/` env (Python 3.8) is still installed for the original benchmark, but the seated
scene does not depend on it at runtime.

## Seated scene (mjlab env, `kangaroo_seated/`)

- Kangaroo from pal_mjlab's `KANGAROO_HANDS_XML`, **free joint removed → fixed base** at the
  setup's base pose; joints set from `seated_arms_at_side`.
- Legs and trunk (`pelvis_*`) must not sag; arm and hand actuators stay free for the team.
- Exposed for the team: a function returning an mjlab `SceneCfg` with entities `robot`
  (Kangaroo), `chair`, `table`, `room` (LIBERO floor/walls), `cup`, `bottle`, `bowl`, plus a plain `MjSpec` builder for
  viewing/debugging without mjlab's env machinery.
- A viewer script with **no task arguments**: `python view.py` (live window) and
  `python view.py --video out/kangaroo.mp4 --seconds 5` (headless).

## Smoke test

1. mjlab env: `mjlab==1.6.0`, `mujoco 3.11.x`, `mujoco_warp 3.11.x`; pal_mjlab imports.
2. LIBERO env: imports without prompting; a LIBERO task resets headless.
3. Seated scene: compiles under MuJoCo 3.11; ~1 s of CPU stepping keeps legs within 0.05 rad of
   target, chair contact present, the three objects stay on the table, an arm moves under command;
   writes `out/kangaroo_seated.png`.
4. Reach: each of cup, bottle, bowl is reachable by at least one hand (palm brought within
   0.1 m by driving the arm, e.g. simple IK/position targets).
5. With a GPU: the scene steps under MuJoCo Warp (skipped otherwise).

## Non-goals

- No mjlab task (rewards, observations, terminations, RL configs), no training.
- No BDDL goal checking, no LIBERO tasks or demos, no scene variation/randomisation, no
  sit-to-stand, no ROS.
- No changes inside the `pal_mjlab/` or `LIBERO/` clones.

## Anti-goals (failure even if it runs)

- Any legacy or unofficial package, or MuJoCo other than 3.11.x in the mjlab env.
- LIBERO and mjlab dependencies mixed in one Python environment.
- A seated pose that exceeds Kangaroo's real joint limits.
- A container that waits for input at import, or whose installed envs are hidden by the mount.

## Settled after builder review (my calls unless marked)

- **Lock vs pins (answered):** pal_mjlab's `uv.lock` at `52f5d6ed5d` resolves to `mjlab 1.6.0`,
  `mujoco 3.11.0`, `mujoco-warp 3.11.0`, `torch 2.10.0`, so no conflict; install `--frozen`.
- **Objects are loose and graspable**, each its own mjlab entity.
- **CPU fallback:** build arg `TORCH_EXTRA=cu128` (default) or `cpu`; the README shows running
  without `--gpus all`.
- **Simplifying collision for MuJoCo Warp:** allowed only if Warp rejects the LIBERO geometry.
  Visual meshes stay; collision may swap to convex hulls or primitives. Smoke check 5 documents it.

## Left to the builder

- Leg-holding mechanism (position-actuator targets vs. equality joint locks at `solref 2*dt`).
- Whether the LIBERO env needs torch at all for scene export (skip the CUDA torch if not).
- Exact object x/y positions (within the constraints above).
- How the `SceneCfg` places a fixed-base entity and a static XML scene (mjlab specifics).
