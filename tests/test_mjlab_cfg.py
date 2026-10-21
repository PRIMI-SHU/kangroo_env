import pytest, torch
from kangaroo_seated import make_scene_cfg, POSE, HELD_JOINTS

@pytest.mark.skipif(not torch.cuda.is_available(), reason="no GPU: MuJoCo Warp check skipped")
def test_scene_steps_on_warp():
    from mjlab.scene import Scene
    from mjlab.sim import Simulation, SimulationCfg
    scene = Scene(make_scene_cfg(num_envs=4), device="cuda:0")
    sim = Simulation(num_envs=4, cfg=SimulationCfg(nconmax=128, njmax=600), spec=scene.spec, device="cuda:0")
    scene.initialize(sim.mj_model, sim.model, sim.data)
    sim.reset(); scene.reset()
    # what a task's reset_scene_to_default event + zero joint-position action (default offset) do
    for e in scene.entities.values():
        if not e.is_fixed_base:
            e.write_root_state_to_sim(e.data.default_root_state)
        if e.is_articulated:
            e.write_joint_state_to_sim(e.data.default_joint_pos, e.data.default_joint_vel)
        if e.is_actuated:
            e.set_joint_position_target(e.data.default_joint_pos)
    sim.forward()
    robot = scene["robot"]
    for _ in range(100):
        scene.write_data_to_sim(); sim.step(); scene.update(sim.mj_model.opt.timestep)
    q = robot.data.joint_pos
    assert torch.isfinite(q).all()
    ids, names = robot.find_joints(HELD_JOINTS, preserve_order=True)
    err = (q[:, ids] - torch.tensor([POSE[n] for n in names], device=q.device)).abs().max()
    assert err < 0.05, err.item()
