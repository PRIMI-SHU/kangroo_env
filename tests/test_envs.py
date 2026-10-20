import pathlib, subprocess

ROOT = pathlib.Path(__file__).resolve().parents[1]  # /opt/robot-learning in the image

def test_clones_present():
    for d in ("pal_mjlab", "LIBERO"):
        assert (ROOT / d).is_dir(), f"missing {d}/: run ./setup.sh on the host"

def test_assets_present():
    for p in ("cup/model.xml", "cup/model.obj", "cup/texture.png",
              "apple/textured.obj", "apple/texture_map.png",
              "bottle/objects/bottle.xml", "bottle/objects/meshes/bottle.stl", "bottle/textures/glass.png"):
        f = ROOT / "assets" / p
        assert f.is_file() and f.stat().st_size > 0, f"missing/empty assets/{p}: run ./setup.sh on the host"

def test_ros_humble_python():
    import os, sys
    assert sys.version_info[:2] == (3, 10)
    assert os.environ.get("ROS_DISTRO") == "humble", "ROS not sourced"
    import rclpy  # noqa: F401  ROS 2 is importable from the mjlab venv

def test_mjlab_stack_versions():
    import mujoco, importlib.metadata as md
    assert mujoco.__version__.startswith("3.11.")
    assert md.version("mujoco-warp").startswith("3.11.")
    assert md.version("mjlab") == "1.6.0"
    from pal_mjlab.robots.pal_kangaroo.kangaroo_constants import KANGAROO_HANDS_XML
    assert KANGAROO_HANDS_XML.exists()

def test_editable_paths_point_at_root():
    import pal_mjlab
    assert pal_mjlab.__file__.startswith(f"{ROOT}/pal_mjlab/")
    out = subprocess.run(["/opt/venvs/libero/bin/python", "-c", "import sys, libero.libero as l; assert sys.version_info[:2] == (3, 10); print(l.__file__)"],
                         capture_output=True, text=True, stdin=subprocess.DEVNULL, timeout=120)
    assert out.stdout.startswith(f"{ROOT}/LIBERO/"), out.stderr

def test_libero_imports_without_prompt_and_resets():
    # stdin=DEVNULL: an input() prompt raises EOFError and fails the test.
    code = ("import os; from libero.libero import benchmark, get_libero_path;"
            "from libero.libero.envs import OffScreenRenderEnv;"
            "t=benchmark.get_benchmark_dict()['libero_spatial']().get_task(0);"
            "e=OffScreenRenderEnv(bddl_file_name=os.path.join(get_libero_path('bddl_files'),t.problem_folder,t.bddl_file),"
            "camera_heights=64,camera_widths=64); e.reset(); e.close(); print('OK')")
    out = subprocess.run(["/opt/venvs/libero/bin/python", "-c", code], capture_output=True, text=True,
                         stdin=subprocess.DEVNULL, timeout=600)
    assert "OK" in out.stdout, out.stderr[-2000:]
