"""View the seated Kangaroo scene: `python -m kangaroo_seated.view` (live window) or `... --video out.mp4` (headless).

Arms are held by pal_mjlab's position actuators with both hands resting on the table.
"""
import argparse, os, time, imageio, mujoco
from kangaroo_seated import make_model

LOOKAT, DISTANCE, AZIMUTH, ELEVATION = [-0.1, 0, 0.75], 2.2, 90, -10  # in-room camera (LIBERO walls make the default far away)

p = argparse.ArgumentParser()
p.add_argument("--video"); p.add_argument("--seconds", type=float, default=5)
a = p.parse_args()
m, d = make_model()
if a.video:
    os.makedirs(os.path.dirname(a.video) or ".", exist_ok=True)
    cam = mujoco.MjvCamera(); cam.lookat[:] = LOOKAT; cam.distance, cam.azimuth, cam.elevation = DISTANCE, AZIMUTH, ELEVATION
    with mujoco.Renderer(m, 480, 640) as r, imageio.get_writer(a.video, fps=30) as w:
        for _ in range(int(a.seconds * 30)):
            mujoco.mj_step(m, d, nstep=round(1 / 30 / m.opt.timestep))
            r.update_scene(d, camera=cam); w.append_data(r.render())
else:
    import mujoco.viewer
    with mujoco.viewer.launch_passive(m, d) as v:
        v.cam.lookat[:] = LOOKAT; v.cam.distance, v.cam.azimuth, v.cam.elevation = DISTANCE, AZIMUTH, ELEVATION
        while v.is_running():
            t = time.time(); mujoco.mj_step(m, d); v.sync()
            time.sleep(max(0, m.opt.timestep - (time.time() - t)))
