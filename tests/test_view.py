import subprocess, sys

def test_view_video(tmp_path):
    out = tmp_path / "t.mp4"  # run from an unrelated folder: the viewer ships inside the package
    r = subprocess.run([sys.executable, "-m", "kangaroo_seated.view", "--video", str(out), "--seconds", "1"],
                       capture_output=True, text=True, cwd=tmp_path)
    assert r.returncode == 0, r.stderr[-500:]
    assert out.stat().st_size > 1000
