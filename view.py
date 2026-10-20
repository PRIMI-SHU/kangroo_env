"""Watch the combined scene: live interactive viewer (default) or a rendered video.

  python view.py                                   # live window (needs X11, see README)
  python view.py --video out/kangaroo.mp4          # headless: render a video instead
  python view.py --suite libero_object --task 3    # any LIBERO suite / task
  python view.py --seated                          # Kangaroo on a chair at the table
"""
import argparse
import os

import imageio
import mujoco

from kangaroo_libero import make_scene

p = argparse.ArgumentParser()
p.add_argument("--suite", default="libero_spatial")
p.add_argument("--task", type=int, default=0)
p.add_argument("--video", help="write an mp4 here instead of opening a window")
p.add_argument("--seconds", type=float, default=5.0)
p.add_argument("--camera", default="frontview")
p.add_argument("--seated", action="store_true", help="Kangaroo sits on a chair at the table")
args = p.parse_args()

m, d, language = make_scene(args.suite, task_id=args.task, seated=args.seated)
print("Task:", language, flush=True)

if args.video:
    fps = 30
    steps_per_frame = round(1 / (fps * m.opt.timestep))
    with mujoco.Renderer(m, 480, 640) as r:
        frames = []
        for _ in range(int(args.seconds * fps)):
            mujoco.mj_step(m, d, nstep=steps_per_frame)
            r.update_scene(d, camera=args.camera)
            frames.append(r.render())
    os.makedirs(os.path.dirname(args.video) or ".", exist_ok=True)
    imageio.mimsave(args.video, frames, fps=fps)
    print("Wrote", args.video, flush=True)
else:
    import mujoco.viewer
    mujoco.viewer.launch(m, d)  # blocks until the window is closed; Space pauses, Backspace resets

# Skip interpreter teardown: robosuite's EGL context and GLFW otherwise segfault on exit.
os._exit(0)
