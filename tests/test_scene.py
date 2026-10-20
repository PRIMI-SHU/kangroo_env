import copy, re, subprocess, sys, numpy as np, mujoco
from kangaroo_seated import POSE, HELD_JOINTS, OBJECTS, make_model
from kangaroo_seated.scene import SETUP, lowest_z

def test_import_is_silent():
    r = subprocess.run([sys.executable, "-c", "import kangaroo_seated"], capture_output=True, text=True)
    assert r.returncode == 0 and "WARN" not in r.stdout + r.stderr and "Traceback" not in r.stdout + r.stderr, r.stderr[-300:]

def step_1s(m, d): mujoco.mj_step(m, d, nstep=int(1.0 / m.opt.timestep))

def bodies(m, c): return {m.body(m.geom_bodyid[c.geom1]).name, m.body(m.geom_bodyid[c.geom2]).name}

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
        assert abs(d.joint(f"robot/{n}").qpos[0] - q) < 1e-9, n  # make_model starts at the pose

def test_seated_scene_holds():
    m, d = make_model(); step_1s(m, d)
    assert np.isfinite(d.qpos).all()
    assert max(abs(d.joint(f"robot/{n}").qpos[0] - POSE[n]) for n in HELD_JOINTS) < 0.05
    assert any("chair/chair" in bodies(m, c) for c in d.contact[:d.ncon])

def test_objects_stay_on_table():
    m, d = make_model()
    z0 = {o: d.body(f"{o}/{o}").xpos[2] for o in OBJECTS}
    step_1s(m, d)
    for o in OBJECTS:
        assert abs(d.body(f"{o}/{o}").xpos[2] - z0[o]) < 0.02, o
        assert np.linalg.norm(d.body(f"{o}/{o}").cvel) < 0.05, o

def test_arms_held_with_hands_on_table():
    m, d = make_model(); step_1s(m, d)
    arms = [n for n in POSE if n.startswith("arm_")]
    assert len(arms) == 10 and all(m.actuator(f"robot/{n}") for n in arms)  # PAL's arm actuators, not free
    assert max(abs(d.joint(f"robot/{n}").qpos[0] - POSE[n]) for n in arms) < 0.1  # table contact: test_hands_rest_on_table_not_in_it

def test_hands_rest_on_table_not_in_it():
    """Visible forearm and hand meshes collide; each hand touches the table top and nothing sinks into it."""
    m, d = make_model()
    for t in ("start", "settled"):
        if t == "settled":
            step_1s(m, d)
        touching = {"left": [], "right": []}
        for c in d.contact[:d.ncon]:
            names = bodies(m, c)
            arm = next((n for n in names if n.startswith(("robot/arm_", "robot/hand_"))), None)
            if "table/table" in names and arm:
                assert c.dist > -0.002, (t, arm, c.dist)
                touching["left" if "left" in arm else "right"].append(arm)
        if t == "settled":
            for s in ("left", "right"):  # the hand or its wrist (arm_*_5, which carries the hand) is on the table
                assert any("hand_" in n or "_5_link" in n for n in touching[s]), (s, touching[s])
                hand = [g for g in range(m.ngeom) if re.fullmatch(f"robot/hand_{s}.*_collision", m.geom(g).name)]
                assert lowest_z(m, d, hand) - SETUP["table"]["top_z"] < 0.003, s  # no hand hovering above the top

def test_arm_moves():
    m, d = make_model()
    j = m.joint("robot/arm_left_1_joint"); q0 = d.qpos[j.qposadr[0]]
    d.ctrl[m.actuator("robot/arm_left_1_joint").id] = q0 + 0.3  # position target
    mujoco.mj_step(m, d, nstep=500)
    assert abs(d.qpos[j.qposadr[0]] - q0) > 0.05

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
        best = min(min_palm_distance(m, d, s, d.body(f"{o}/{o}").xpos.copy()) for s in ("left", "right"))
        assert best < 0.10, (o, best)
