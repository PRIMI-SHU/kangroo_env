"""Kangaroo seated on a chair at a table, with a cup, bottle and bowl (MjSpec; no actuators, arms are passive)."""
import json
from pathlib import Path

import mjlab  # noqa: F401  (first: its entry-point loader imports pal_mjlab.tasks, which is circular if pal_mjlab loads first)
import mujoco
import numpy as np
from pal_mjlab.robots.pal_kangaroo.kangaroo_constants import KANGAROO_HANDS_XML

ROOM = "/workspace/LIBERO/libero/libero/assets/scenes/libero_tabletop_base_style.xml"
ASSETS = Path("/workspace/assets")
OBJECT_XML = {"cup": ASSETS / "cup/model.xml", "bowl": ASSETS / "bowl/model.xml",
              "bottle": ASSETS / "bottle/objects/bottle.xml"}
OBJECT_BODY = {"cup": "model", "bowl": "model", "bottle": "object"}  # body of each XML that gets attached
OBJECTS = ("cup", "bottle", "bowl")
OBJECT_X = -0.05          # Task 3 tunes
OBJECT_DY = 0.15          # spacing in y, centred on y = 0
SETUP = json.load(open(Path(__file__).with_name("seated_setup.json")))
POSE = SETUP["poses"]["seated_arms_at_side"]["joints"]
HELD_JOINTS = [n for n in POSE if n.startswith(("leg_", "pelvis_")) and not n.endswith(("_femur_joint", "_knee_joint"))]


def room_spec():
    spec = mujoco.MjSpec.from_file(ROOM)
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
    """kangaroo_hands without keyframes or free joint, base_link at the origin (the parent frame carries the setup pose)."""
    robot = mujoco.MjSpec.from_file(str(KANGAROO_HANDS_XML))
    for k in list(robot.keys):
        robot.delete(k)
    base = robot.body("base_link")
    for j in base.joints:
        robot.delete(j)  # free joint 'origin'
    base.pos, base.quat = [0, 0, 0], [1, 0, 0, 0]
    return robot


def object_spec(name):
    """Free-floating body `name` at the origin wrapping the object's XML body."""
    spec = mujoco.MjSpec()
    body = spec.worldbody.add_body(name=name)
    body.add_freejoint()
    body.add_frame().attach_body(mujoco.MjSpec.from_file(str(OBJECT_XML[name])).body(OBJECT_BODY[name]), name + "_", "")
    return spec


def object_pos(name, object_z=None):
    return [OBJECT_X, (OBJECTS.index(name) - 1) * OBJECT_DY, SETUP["table"]["top_z"] + (object_z or {}).get(name, 0.0)]


def lock_held_joints(spec, prefix="robot/"):
    for n in HELD_JOINTS:
        spec.add_equality(type=mujoco.mjtEq.mjEQ_JOINT, name1=prefix + n, data=[POSE[n]] + [0] * 10,
                          solref=[2 * spec.option.timestep, 1])


def build_spec(object_z=None):
    spec = room_spec()
    for k in ("chair", "table"):
        spec.attach(furniture_spec(k), prefix="", frame=spec.worldbody.add_frame())
    spec.attach(robot_spec(), prefix="robot/",
                frame=spec.worldbody.add_frame(pos=SETUP["base_link"]["pos"], quat=SETUP["base_link"]["quat_wxyz"]))
    for o in OBJECTS:
        spec.attach(object_spec(o), prefix="", frame=spec.worldbody.add_frame(pos=object_pos(o, object_z)))
    lock_held_joints(spec)
    return spec


def _rest_heights():
    """Height of each object's origin above the table top so its lowest collision vertex touches it."""
    m = build_spec().compile()
    d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    low = {}
    for g in range(m.ngeom):
        o = next((o for o in OBJECTS if m.body(m.geom_bodyid[g]).name.startswith(o)), None)
        if o is None or not m.geom_contype[g] or m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        a, n = m.mesh_vertadr[m.geom_dataid[g]], m.mesh_vertnum[m.geom_dataid[g]]
        z = (d.geom_xpos[g] + m.mesh_vert[a:a + n] @ d.geom_xmat[g].reshape(3, 3).T)[:, 2].min()
        low[o] = min(low.get(o, np.inf), z)
    return {o: d.body(o).xpos[2] - z + 0.001 for o, z in low.items()}  # 1 mm above the table


def make_model():
    m = build_spec(_rest_heights()).compile()
    d = mujoco.MjData(m)
    for n, q in POSE.items():
        d.joint(f"robot/{n}").qpos[0] = q
    mujoco.mj_forward(m, d)
    return m, d
