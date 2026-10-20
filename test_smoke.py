"""Smoke test: proves the environment works end to end. Run inside the image: `pytest -v`."""
import os


def test_pinned_stack():
    import mujoco
    import robosuite
    from libero.libero import benchmark  # would block on input() if ~/.libero/config.yaml were missing

    assert mujoco.__version__.startswith("3.8."), mujoco.__version__
    assert robosuite.__version__ == "1.4.0", robosuite.__version__
    assert "libero_spatial" in benchmark.get_benchmark_dict()


def test_libero_task_runs_headless():
    import os
    from libero.libero import benchmark, get_libero_path
    from libero.libero.envs import OffScreenRenderEnv

    task = benchmark.get_benchmark_dict()["libero_spatial"]().get_task(0)
    bddl = os.path.join(get_libero_path("bddl_files"), task.problem_folder, task.bddl_file)
    env = OffScreenRenderEnv(bddl_file_name=bddl, camera_heights=128, camera_widths=128)
    env.seed(0)
    env.reset()
    for _ in range(20):
        obs, _, _, _ = env.step([0.0] * 7)  # stock Panda: 6-DoF OSC delta pose + gripper
    env.close()
    assert obs["agentview_image"].shape == (128, 128, 3)
    assert obs["agentview_image"].max() > 0  # not a black frame: rendering actually happened


def test_kangaroo_loads_and_steps():
    import mujoco
    import numpy as np
    from kangaroo_libero import kangaroo_scene_xml

    m = mujoco.MjModel.from_xml_path(kangaroo_scene_xml())
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d, nstep=1000)
    assert m.nu == 38  # PAL's 5-DoF-arm + RH8D-hand variant, counted from its MJCF
    assert np.isfinite(d.qpos).all()


def test_kangaroo_in_libero_scene_steps_and_renders():
    import mujoco
    import numpy as np
    from PIL import Image
    from kangaroo_libero import make_scene

    m, d, language = make_scene("libero_spatial", task_id=0)
    assert language.startswith("pick up the black bowl")
    assert m.nu == 38  # Kangaroo's actuators only: the Panda's were removed
    assert d.joint("plate_1_joint0").qpos[2] > 0.85  # LIBERO object placed on the table top (~0.9 m)

    mujoco.mj_step(m, d, nstep=500)
    assert np.isfinite(d.qpos).all()
    assert abs(d.body("kangaroo/base_link").xpos[2] - 0.9) < 1e-6  # welded base: did not fall

    with mujoco.Renderer(m, 480, 640) as r:
        r.update_scene(d, camera="frontview")
        img = r.render()
    assert img.max() > 0
    os.makedirs("out", exist_ok=True)
    Image.fromarray(img).save("out/kangaroo_libero.png")


def test_kangaroo_seated_on_chair():
    import mujoco
    import numpy as np
    from PIL import Image
    from kangaroo_libero import SEATED_BASE_POS, SEATED_LEGS, make_scene

    m, d, _ = make_scene("libero_spatial", task_id=0, seated=True)
    assert m.nu == 38  # same actuator layout as standing; leg actuators are just overridden
    for name in SEATED_LEGS:  # seated pose respects PAL's real joint limits
        lo, hi = m.joint(f"kangaroo/{name}").range
        assert lo <= SEATED_LEGS[name] <= hi, name

    arm = "kangaroo/arm_left_1_joint"
    q_arm = d.joint(arm).qpos.copy()
    d.actuator("kangaroo/arm_left_1_actuator_motor").ctrl = 20.0
    mujoco.mj_step(m, d, nstep=500)
    assert np.isfinite(d.qpos).all()
    assert abs(d.body("kangaroo/base_link").xpos[2] - SEATED_BASE_POS[2]) < 1e-6  # still welded
    drift = max(abs(d.joint(f"kangaroo/{n}").qpos[0] - q) for n, q in SEATED_LEGS.items())
    assert drift < 0.05, drift  # locked legs hold the seated pose
    assert any(m.body(m.geom_bodyid[g]).name == "chair" for c in d.contact[:d.ncon] for g in (c.geom1, c.geom2))
    assert abs(d.joint(arm).qpos[0] - q_arm[0]) > 0.05  # arms still free for the policy

    cam = mujoco.MjvCamera()
    cam.lookat[:], cam.distance, cam.azimuth, cam.elevation = (-0.75, 0, 0.6), 2.4, 90, -10
    with mujoco.Renderer(m, 480, 640) as r:
        r.update_scene(d, camera=cam)
        img = r.render()
    os.makedirs("out", exist_ok=True)
    Image.fromarray(img).save("out/kangaroo_seated.png")
