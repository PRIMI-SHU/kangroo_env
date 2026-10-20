"""The seated scene as an mjlab SceneCfg (task-free: entities only, no managers)."""
import mjlab  # noqa: F401  (before pal_mjlab, see scene.py)
from mjlab.entity import EntityCfg
from mjlab.scene import SceneCfg
from pal_mjlab.robots.pal_kangaroo.kangaroo_constants import get_kangaroo_hands_robot_cfg

from .scene import OBJECTS, POSE, SETUP, _rest_heights, furniture_spec, lock_held_joints, object_pos, \
    object_spec, robot_spec, room_spec


def robot_cfg():
    """pal_mjlab's kangaroo_hands entity (actuators, collisions), fixed base at the setup pose, joints at POSE."""
    cfg = get_kangaroo_hands_robot_cfg()
    cfg.spec_fn = robot_spec
    cfg.init_state = EntityCfg.InitialStateCfg(  # new object: pal_mjlab shares INIT_STATE between variants
        pos=tuple(SETUP["base_link"]["pos"]), rot=tuple(SETUP["base_link"]["quat_wxyz"]), joint_pos=dict(POSE))
    return cfg


def make_scene_cfg(num_envs: int = 1) -> SceneCfg:
    z = _rest_heights()
    return SceneCfg(
        num_envs=num_envs,
        entities={
            "room": EntityCfg(spec_fn=room_spec),
            "chair": EntityCfg(spec_fn=lambda: furniture_spec("chair")),
            "table": EntityCfg(spec_fn=lambda: furniture_spec("table")),
            "robot": robot_cfg(),
            **{o: EntityCfg(spec_fn=lambda o=o: object_spec(o),
                            init_state=EntityCfg.InitialStateCfg(pos=tuple(object_pos(o, z))))
               for o in OBJECTS},
        },
        spec_fn=lock_held_joints,
    )
