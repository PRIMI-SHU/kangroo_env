"""View the seated Kangaroo scene: live window by default, or --video out.mp4 headless."""
import argparse, imageio, mujoco
from kangaroo_seated import make_model

p = argparse.ArgumentParser()
p.add_argument("--video"); p.add_argument("--seconds", type=float, default=5)
a = p.parse_args()
m, d = make_model()
if a.video:
    cam = mujoco.MjvCamera(); cam.lookat[:] = [-0.1, 0, 0.75]; cam.distance, cam.azimuth, cam.elevation = 2.2, 90, -10
    with mujoco.Renderer(m, 480, 640) as r, imageio.get_writer(a.video, fps=30) as w:
        for _ in range(int(a.seconds * 30)):
            mujoco.mj_step(m, d, nstep=round(1 / 30 / m.opt.timestep))
            r.update_scene(d, camera=cam); w.append_data(r.render())
else:
    import mujoco.viewer
    mujoco.viewer.launch(m, d)
