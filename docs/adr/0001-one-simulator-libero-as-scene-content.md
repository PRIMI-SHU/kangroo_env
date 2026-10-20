# One simulator; LIBERO contributes scene content, not code

LIBERO and Kangaroo appear together in one MuJoCo scene run by mjlab (MuJoCo 3.11). LIBERO's Python
stack lives in its own venv and is never imported into the mjlab venv, because LIBERO pins
`numpy==1.22.4` and `robosuite==1.4.0` while mjlab needs numpy 2, and robosuite has no Kangaroo robot.
LIBERO contributes geometry as MuJoCo files (today its room; later objects, fixtures or task layouts
exported from the `libero` venv and loaded by mjlab). Both venvs use ROS 2 Humble's system Python 3.10.

## Considered Options

- One venv with LIBERO and mjlab: impossible without unpinning LIBERO (unofficial, untested).
- Kangaroo inside robosuite/LIBERO: needs a robosuite robot port and MuJoCo 3.2, losing mjlab and MuJoCo Warp.
