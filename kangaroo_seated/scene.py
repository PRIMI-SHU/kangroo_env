"""The seated scene: Kangaroo on a chair at a table, hands resting on it, with a cup, bottle and apple.

Promised interface (README "Promised interface"): make_scene_cfg(num_envs) -> mjlab SceneCfg, make_model() ->
plain-MuJoCo (model, data) at the start pose compiled from that same SceneCfg, and POSE, HELD_JOINTS, OBJECTS. Entities:
robot, room, chair, table, cup, bottle, apple; mjlab prefixes names with the entity ("robot/base_link", "cup/cup").
Arms, legs and trunk use pal_mjlab's own position actuators (kangaroo_constants COMMON_ACTUATORS); ctrl = target angles.
"""
import functools
import json
import re
from pathlib import Path

import mjlab  # noqa: F401  (first: its entry-point loader imports pal_mjlab.tasks, which is circular if pal_mjlab loads first)
import mujoco
import numpy as np
from mjlab.entity import EntityCfg
from mjlab.scene import Scene, SceneCfg
from pal_mjlab.robots.pal_kangaroo.kangaroo_constants import KANGAROO_HANDS_XML, get_kangaroo_hands_robot_cfg

ROOT = Path(__file__).resolve().parents[1]  # pal_mjlab/, LIBERO/ and assets/ sit next to this package (/opt/robot-learning)
ROOM = ROOT / "LIBERO/libero/libero/assets/scenes/libero_tabletop_base_style.xml"
ASSETS = ROOT / "assets"
OBJECT_XML = {"cup": ASSETS / "cup/model.xml", "apple": Path(__file__).with_name("apple.xml"),
              "bottle": ASSETS / "bottle/objects/bottle.xml"}
OBJECT_BODY = {"cup": "model", "apple": "apple", "bottle": "object"}  # body of each XML that gets attached
OBJECTS = ("cup", "bottle", "apple")
OBJECT_X = 0.10           # 40 cm from the table's front edge: ~80% of the palm's reach (reach runs out at ~50 cm)
OBJECT_DY = 0.15          # spacing in y, centred on y = 0
SETUP = json.load(open(Path(__file__).with_name("seated_setup.json")))
# Confirmed seated_hands_on_table pose (same legs as seated_arms_at_side). With collision on the visible meshes it puts the
# left fingertips 25 mm into the table and the right hand 20 mm above it (forearm 1.5 mm in). Smallest changes that start
# each hand ~0.5 mm above the top (tests/test_scene.py checks the contacts):
POSE = {**SETUP["poses"]["seated_hands_on_table"]["joints"],
        "arm_left_4_joint": 1.557591,    # elbow +0.072 rad: fingertips on the table, forearm 29 mm above it
        "arm_right_2_joint": 0.922775,   # shoulder -0.025 rad and
        "arm_right_4_joint": 1.842134}   # elbow -0.047 rad: right hand and forearm both on the table
FOREARM_AND_HAND = re.compile(r"(arm_(left|right)_[45]|hand_(left|right)_.*)_link")
HELD_JOINTS = [n for n in POSE if n.startswith(("leg_", "pelvis_")) and not n.endswith(("_femur_joint", "_knee_joint"))]


def room_spec():
    spec = mujoco.MjSpec.from_file(str(ROOM))
    spec.delete(spec.body("table"))  # LIBERO's table; ours comes from the setup
    return spec


def furniture_spec(name):
    """Static 'chair' or 'table' body from the setup boxes (world coordinates), so contacts can be identified by body name."""
    spec = mujoco.MjSpec()
    body = spec.worldbody.add_body(name=name)
    for b in SETUP[name]["boxes"]:
        body.add_geom(name=b["name"], type=mujoco.mjtGeom.mjGEOM_BOX, size=b["size_half"],
                      pos=b["pos"], rgba=b["rgba"], contype=1, conaffinity=1, group=0)
    return spec


def robot_spec():
    """kangaroo_hands without keyframes or free joint, base_link at the origin (the entity's init_state places it)."""
    robot = mujoco.MjSpec.from_file(str(KANGAROO_HANDS_XML))
    for k in list(robot.keys):
        robot.delete(k)
    base = robot.body("base_link")
    for j in base.joints:
        robot.delete(j)  # free joint 'origin'
    base.pos, base.quat = [0, 0, 0], [1, 0, 0, 0]
    # pal_mjlab's hands have no collision geoms and the forearm collides as a coarse 7 cm capsule, so on the table
    # the fingers sink in and the capsule props the hand up. Collide with the visible meshes instead (mjlab's
    # CollisionCfg only keeps geoms named *_collision).
    for s in ("left", "right"):
        robot.delete(robot.geom(f"arm_{s}_4_collision"))
    for i, g in enumerate(robot.geoms):
        if g.type == mujoco.mjtGeom.mjGEOM_MESH and FOREARM_AND_HAND.fullmatch(g.parent.name):
            g.name = f"{g.parent.name}_mesh{i}_collision"
    return robot


def robot_cfg():
    """pal_mjlab's kangaroo_hands entity (actuators, collisions), fixed base at the setup pose, joints at POSE."""
    cfg = get_kangaroo_hands_robot_cfg()
    cfg.spec_fn = robot_spec
    cfg.init_state = EntityCfg.InitialStateCfg(  # new object: pal_mjlab shares INIT_STATE between variants
        pos=tuple(SETUP["base_link"]["pos"]), rot=tuple(SETUP["base_link"]["quat_wxyz"]), joint_pos=dict(POSE))
    return cfg


def object_spec(name):
    """Free-floating body `name` at the origin wrapping the object's XML body."""
    spec = mujoco.MjSpec()
    body = spec.worldbody.add_body(name=name)
    body.add_freejoint()
    body.add_frame().attach_body(mujoco.MjSpec.from_file(str(OBJECT_XML[name])).body(OBJECT_BODY[name]), name + "_", "")
    return spec


def lowest_z(m, d, geoms):
    """Lowest world z over the vertices of the given mesh geoms (mj_geomDistance is unreliable between meshes)."""
    def low(g):
        a, n = m.mesh_vertadr[m.geom_dataid[g]], m.mesh_vertnum[m.geom_dataid[g]]
        return (d.geom_xpos[g] + m.mesh_vert[a:a + n] @ d.geom_xmat[g].reshape(3, 3).T)[:, 2].min()
    return min(low(g) for g in geoms)


@functools.cache
def object_pos(name):
    """Start position: in the row on the table, origin raised so the lowest collision vertex sits 1 mm above the top."""
    m = object_spec(name).compile()
    d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    low = lowest_z(m, d, [g for g in range(m.ngeom) if m.geom_contype[g] and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH])
    return OBJECT_X, (OBJECTS.index(name) - 1) * OBJECT_DY, SETUP["table"]["top_z"] - low + 0.001


def lock_held_joints(spec):
    for n in HELD_JOINTS:
        spec.add_equality(type=mujoco.mjtEq.mjEQ_JOINT, name1="robot/" + n, data=[POSE[n]] + [0] * 10,
                          solref=[2 * spec.option.timestep, 1])


def make_scene_cfg(num_envs: int = 1) -> SceneCfg:
    """The seated scene as an mjlab SceneCfg (task-free: entities only, no managers)."""
    return SceneCfg(
        num_envs=num_envs,
        entities={
            "room": EntityCfg(spec_fn=room_spec),
            "chair": EntityCfg(spec_fn=lambda: furniture_spec("chair")),
            "table": EntityCfg(spec_fn=lambda: furniture_spec("table")),
            "robot": robot_cfg(),
            **{o: EntityCfg(spec_fn=lambda o=o: object_spec(o), init_state=EntityCfg.InitialStateCfg(pos=object_pos(o)))
               for o in OBJECTS},
        },
        spec_fn=lock_held_joints,
    )


def make_model():
    """Plain-MuJoCo (model, data) of the same scene, reset to mjlab's start keyframe (pose, object positions, ctrl targets)."""
    m = Scene(make_scene_cfg(), device="cpu").spec.compile()
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, m.key("init_state").id)
    mujoco.mj_forward(m, d)
    return m, d
