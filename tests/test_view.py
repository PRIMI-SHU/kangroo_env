import os, subprocess, sys

def test_view_video(tmp_path):
    out = tmp_path / "t.mp4"
    r = subprocess.run([sys.executable, "view.py", "--video", str(out), "--seconds", "1"], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr[-500:]
    assert out.stat().st_size > 1000
