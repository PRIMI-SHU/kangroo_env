# Kangaroo LIBERO Environment

A ready-to-run simulation environment that puts PAL Robotics' Kangaroo humanoid into LIBERO's manipulation scenes, so the team can train RL policies on it. This project provides the environment only; the RL algorithms belong to the team.

## Language

### Robot

**Kangaroo**:
PAL Robotics' bipedal humanoid, as modelled in PAL's public `kangaroo_mujoco` package.
_Avoid_: the robot, PAL robot, humanoid

**Kangaroo variant**:
One of PAL's pre-generated Kangaroo models, differing in base (floating or fixed), arm DoF and end-effector (e.g. `floating_5dof_RH8D_RH8D_fixed`: 5-DoF arms with RH8D hands).
_Avoid_: config, version, flavour

**Welded base**:
The Kangaroo pelvis rigidly fixed in the world so the robot cannot fall; a simplification that separates manipulation from balance.
_Avoid_: fixed robot, pinned base

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

**Combined scene**:
A Kangaroo variant attached into a LIBERO arena in a single MuJoCo model; physics and cameras work, but LIBERO task logic is not wired to Kangaroo.
_Avoid_: Kangaroo LIBERO task, integrated env

**Smoke test**:
The checks that prove the environment works end to end: pinned versions, LIBERO runs headless, Kangaroo loads, the combined scene steps and renders.
_Avoid_: unit tests, benchmark
