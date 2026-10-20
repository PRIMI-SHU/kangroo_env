import copy, os, subprocess, sys, imageio, numpy as np, mujoco
from kangaroo_seated import POSE, HELD_JOINTS, OBJECTS, make_model

def test_import_is_silent():
    r = subprocess.run([sys.executable, "-c", "import kangaroo_seated"], capture_output=True, text=True)
    assert r.returncode == 0 and "WARN" not in r.stdout + r.stderr and "Traceback" not in r.stdout + r.stderr, r.stderr[-300:]

def step_1s(m, d): mujoco.mj_step(m, d, nstep=int(1.0 / m.opt.timestep))

def test_matches_confirmed_setup():
    m, d = make_model()
    assert np.allclose(d.body("robot/base_link").xpos, [-0.3544, 0, 0.7479], atol=1e-4)
    for s in ("left", "right"):
        # model has no *_sole_link: sole = lowest point of the foot's (sphere/capsule) collision geoms; setup: -0.5 mm
        b = m.body(f"robot/leg_{s}_foot_link").id
        sole = min(d.geom_xpos[g][2] - m.geom_size[g][0] for g in range(m.ngeom) if m.geom_bodyid[g] == b and m.geom_contype[g])
        assert abs(sole) < 0.005
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
    d.qfrc_applied[m.jnt_dofadr[j.id]] = 20.0  # no actuators in kangaroo_hands.xml; apply torque directly
    mujoco.mj_step(m, d, nstep=500)
    assert abs(d.qpos[j.qposadr[0]] - q0) > 0.05
    cam = mujoco.MjvCamera(); cam.lookat[:] = [-0.1, 0, 0.75]  # in-room camera (setup's 3.2 m camera sits behind LIBERO's wall)
    cam.distance, cam.azimuth, cam.elevation = 2.2, 90, -10
    with mujoco.Renderer(m, 480, 640) as r:
        r.update_scene(d, camera=cam); img = r.render()
    assert img.max() > 0
    os.makedirs("out", exist_ok=True); imageio.imwrite("out/kangaroo_seated.png", img)

def min_palm_distance(m, d, side, target):
    """Damped-least-squares IK on the 5 arm joints (kinematics only, on a copy of d); returns palm-target distance."""
    d = copy.copy(d)
    js = [m.joint(f"robot/arm_{side}_{i}_joint") for i in range(1, 6)]
    palm = m.body(f"robot/hand_{side}_palm_link").id
    jp = np.zeros((3, m.nv))
    for _ in range(200):
        mujoco.mj_kinematics(m, d); mujoco.mj_comPos(m, d)
        err = target - d.xpos[palm]
        mujoco.mj_jacBody(m, d, jp, None, palm)
        J = jp[:, [j.dofadr[0] for j in js]]
        dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(3), err)
        for j, x in zip(js, dq):
            d.qpos[j.qposadr[0]] = np.clip(d.qpos[j.qposadr[0]] + x, *j.range)
    mujoco.mj_kinematics(m, d)
    return float(np.linalg.norm(target - d.xpos[palm]))

def test_each_object_reachable():
    m, d = make_model()
    for o in OBJECTS:
        best = min(min_palm_distance(m, d, s, d.body(o).xpos.copy()) for s in ("left", "right"))
        print(o, d.body(o).xpos, best)
        assert best < 0.10, (o, best)
