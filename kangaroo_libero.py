"""PAL Kangaroo in LIBERO scenes, in plain MuJoCo (no ROS)."""
import os

import mujoco

# Copied from PAL's kangaroo_mujoco ROS package at image build time (see Dockerfile).
MODELS_DIR = os.environ.get("KANGAROO_MODELS", "/opt/kangaroo/models")
# Floating base, 5-DoF arms, RH8D hands: the variant with hands, for manipulation.
DEFAULT_VARIANT = "floating_5dof_RH8D_RH8D_fixed"

# Seated pose, all within PAL's joint limits: hip pitch near its flexion limit (-0.74), leg shortened
# (the passive femur/knee then fold the shank down), ankle pitched to keep the soles flat. Tuned by render.
SEATED_LEGS = {
    f"leg_{side}_{n}_joint": q
    for side in ("left", "right")
    for n, q in (("1", 0.0), ("2", -0.7), ("3", 0.0), ("length", 0.45), ("4", 0.4), ("5", 0.0))
}
SEATED_BASE_POS = (-0.9, 0.0, 0.7)
# Pelvis collision boxes reach 0.22 m below base_link; the torso box reaches 0.23 m behind it.
SEAT_BELOW_BASE, BACK_BEHIND_BASE = 0.22, 0.3


def add_chair(spec, base_pos):
    """Static box chair (seat, backrest, four legs) fitted under and behind Kangaroo's pelvis."""
    x, y, z = base_pos
    top = z - SEAT_BELOW_BASE
    sx, sy, sz = 0.2, 0.22, 0.02  # seat half-sizes
    chair = spec.worldbody.add_body(name="chair", pos=[x - BACK_BEHIND_BASE + sx, y, 0])
    box = dict(type=mujoco.mjtGeom.mjGEOM_BOX, rgba=[0.45, 0.3, 0.2, 1])
    chair.add_geom(name="chair_seat", size=[sx, sy, sz], pos=[0, 0, top - sz], **box)
    chair.add_geom(name="chair_back", size=[0.02, sy, 0.25], pos=[-sx, 0, top + 0.25], **box)
    leg_h = (top - 2 * sz) / 2
    for i, (lx, ly) in enumerate([(sx, sy), (sx, -sy), (-sx, sy), (-sx, -sy)]):
        chair.add_geom(name=f"chair_leg{i}", size=[0.02, 0.02, leg_h], pos=[lx * 0.9, ly * 0.9, leg_h], **box)


def kangaroo_scene_xml(variant=DEFAULT_VARIANT):
    """PAL's own scene (Kangaroo on an empty floor) for a Kangaroo variant."""
    return os.path.join(MODELS_DIR, f"mjcf_data_{variant}", "scene_empty.xml")


def make_scene(suite="libero_spatial", task_id=0, variant=DEFAULT_VARIANT, base_pos=None, seated=False):
    """Combined scene: a LIBERO task's arena and objects, its Panda removed, Kangaroo attached.

    Returns (MjModel, MjData, task language). Kangaroo starts in PAL's `home` pose with a
    welded base at `base_pos` (default: 0.9 m, PAL's home pelvis height). LIBERO task logic
    (BDDL goal, success check) is not wired to Kangaroo.

    seated=True: a box chair is added and Kangaroo perches on it, legs locked in a pose within
    the real joint limits (for seated upper-limb work; leg actuators then have no effect).
    """
    if base_pos is None:
        base_pos = SEATED_BASE_POS if seated else (-0.9, 0.0, 0.9)
    from libero.libero import benchmark, get_libero_path
    from libero.libero.envs import OffScreenRenderEnv

    # Build the real LIBERO task once, to get its MJCF and its canonical initial object poses.
    task_suite = benchmark.get_benchmark_dict()[suite]()
    task = task_suite.get_task(task_id)
    bddl = os.path.join(get_libero_path("bddl_files"), task.problem_folder, task.bddl_file)
    env = OffScreenRenderEnv(bddl_file_name=bddl, camera_heights=64, camera_widths=64)
    env.seed(0)
    env.reset()
    env.set_init_state(task_suite.get_task_init_states(task_id)[0])
    lm, ld = env.env.sim.model._model, env.env.sim.data._data
    obj_qpos = {
        mujoco.mj_id2name(lm, mujoco.mjtObj.mjOBJ_JOINT, j): ld.qpos[lm.jnt_qposadr[j]:lm.jnt_qposadr[j] + 7].copy()
        for j in range(lm.njnt) if lm.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE
    }
    spec = mujoco.MjSpec.from_string(env.env.model.get_xml())
    env.close()

    # Remove the stock Panda (body tree, its actuators and its force/torque sensors).
    for a in list(spec.actuators):
        spec.delete(a)
    for s in list(spec.sensors):
        spec.delete(s)
    spec.delete(spec.body("robot0_base"))

    k = mujoco.MjSpec.from_file(os.path.join(MODELS_DIR, f"mjcf_data_{variant}", "mujoco_description_formatted.xml"))
    home = list(k.keys[0].qpos)[7:]  # PAL's `home` keyframe minus the 7 free-joint values
    for j in [j for j in k.joints if j.type == mujoco.mjtJoint.mjJNT_FREE]:
        k.delete(j)  # welded base; keep the free joint instead for balance/locomotion RL
    for key in list(k.keys):
        k.delete(key)  # keyframe sizes would not match the combined model
    joints = [j.name for j in k.joints]
    spec.attach(k, frame=spec.worldbody.add_frame(pos=list(base_pos)), prefix="kangaroo/")
    pose = dict(zip(joints, home))
    if seated:
        pose.update(SEATED_LEGS)
        for name, q in SEATED_LEGS.items():  # hold each actuated leg joint at its seated value
            spec.add_equality(type=mujoco.mjtEq.mjEQ_JOINT, name1=f"kangaroo/{name}", data=[q] + [0] * 10,
                              solref=[2 * spec.option.timestep, 1])  # stiffest stable: legs must not sag
        add_chair(spec, base_pos)

    m = spec.compile()
    d = mujoco.MjData(m)
    for name, q in pose.items():
        d.joint(f"kangaroo/{name}").qpos = q
    for name, q in obj_qpos.items():
        d.joint(name).qpos = q
    if seated:
        # The passive femur/knee close the leg's kinematic loop; let physics settle them once.
        s = mujoco.MjData(m)
        s.qpos[:] = d.qpos
        mujoco.mj_step(m, s, nstep=500)
        for side in ("left", "right"):
            for part in ("femur", "knee"):
                name = f"kangaroo/leg_{side}_{part}_joint"
                d.joint(name).qpos = s.joint(name).qpos
    mujoco.mj_forward(m, d)
    return m, d, task.language
