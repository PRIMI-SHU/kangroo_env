import importlib.metadata, pathlib, subprocess, sys, textwrap

ROOT = pathlib.Path(__file__).resolve().parents[1]

def test_installed_package_with_version():
    import kangaroo_seated
    assert importlib.metadata.version("kangaroo-seated") == "0.1.0"
    assert pathlib.Path(kangaroo_seated.__file__).is_relative_to(ROOT)  # editable: edits under the root are live

def test_other_project_can_use_the_scene(tmp_path):
    """A consuming project in its own folder: only the promised interface, no repo checkout on the path."""
    (tmp_path / "my_task.py").write_text(textwrap.dedent("""
        import mujoco
        from kangaroo_seated import HELD_JOINTS, OBJECTS, POSE, make_model, make_scene_cfg
        assert {"robot", "room", "chair", "table", *OBJECTS} <= set(make_scene_cfg(num_envs=2).entities)
        m, d = make_model()
        mujoco.mj_step(m, d, nstep=200)
        assert abs(d.body("robot/base_link").xpos[2] - 0.7479) < 1e-3
        assert all(abs(d.joint("robot/" + n).qpos[0] - POSE[n]) < 0.05 for n in HELD_JOINTS)
        assert all(d.body(o + "/" + o).xpos[2] > 0.8 for o in OBJECTS)
        print("OK")
    """))
    r = subprocess.run([sys.executable, "my_task.py"], cwd=tmp_path, capture_output=True, text=True)
    assert r.returncode == 0 and "OK" in r.stdout, r.stderr[-1000:]
