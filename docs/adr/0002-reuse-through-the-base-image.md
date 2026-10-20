# Other projects reuse the seated scene through the base image

Other projects build on this one with `FROM robot-learning` and import `kangaroo_seated`; it is not published as a
standalone pip package. The scene needs pal_mjlab and LIBERO at pinned commits, the downloaded object assets and two
incompatible venvs (ADR 0001), none of which a pip install could bring along. So the image holds everything it provides
under `/opt/robot-learning` (clones, assets, the editable-installed package, its tests) and leaves `/workspace` empty for
the consuming project; developers of this repo mount it over `/opt/robot-learning`. The promised interface and its
version (`0.x`, bumped on any change) are listed in the README.
