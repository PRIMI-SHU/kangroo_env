# Robot-learning Environment

A ready-to-run Docker environment with a seated Kangaroo scene (MuJoCo / mjlab) and the original LIBERO, for robot-learning work such as seated upper-limb rehab. This project provides the environment only; the RL algorithms belong to the team.

## Language

### Setup

**Image**:
The single Docker container the team develops in, built on ROS 2 Humble.
_Avoid_: environment, container setup

**Venv**:
One of the two isolated Python installations inside the image: `mjlab` (Kangaroo simulation) or `libero` (original LIBERO).
_Avoid_: environment, env

### Robot

**Kangaroo**:
PAL Robotics' bipedal humanoid, as modelled in PAL's `pal_mjlab`.
_Avoid_: the robot, PAL robot, humanoid

**pal_mjlab**:
PAL Robotics' mjlab package that provides the Kangaroo model, its actuators and the mjlab (MuJoCo Warp) tooling this project builds on.
_Avoid_: kangaroo_mujoco, ROS package

**Fixed base**:
The Kangaroo pelvis held at the seated pose on the chair, with legs and trunk locked, so the robot cannot fall; arms are held by position actuators, hands resting on the table.
_Avoid_: welded base, pinned robot

### LIBERO

**LIBERO**:
A lifelong robot-learning benchmark built on robosuite and MuJoCo, providing scenes, task definitions and human demonstrations.
_Avoid_: the dataset (it is more than a dataset)

**LIBERO task**:
A goal written in BDDL (e.g. "put the bowl on the plate") together with the scene it runs in and its success check.
_Avoid_: episode, level, challenge

**LIBERO demo**:
A recorded human demonstration (HDF5) for a LIBERO task, captured on a Franka Panda arm. Not included in this environment.
_Avoid_: dataset, trajectories

**LIBERO arena**:
The static surroundings of a LIBERO task (floor, walls, table) as MJCF, without a robot.
_Avoid_: world, map, level

### This project

**Seated scene**:
Kangaroo seated on a chair at a table with a cup, water bottle and apple, in plain MuJoCo (`make_model`) or mjlab (`make_scene_cfg`). It has no task, reward or goal. Kangaroo's hands are rigid (pal_mjlab comments out all finger/palm joints): objects can be pushed or scooped, not finger-grasped.
_Avoid_: combined scene, LIBERO task for Kangaroo, environment

**Scene content**:
Geometry (room, objects, furniture, layouts) that LIBERO contributes to the seated scene as MuJoCo files, without LIBERO's Python code running.
_Avoid_: LIBERO environment

**Task**:
Goals, rewards and success checks the team adds on top of the seated scene; none exist yet. Not a LIBERO task.
_Avoid_: scene, environment

**Tests**:
The pytest suite (`pytest -v tests`) that proves the image works: ROS, both venvs, the seated scene, the mjlab scene and the viewer's video path.
_Avoid_: smoke test
