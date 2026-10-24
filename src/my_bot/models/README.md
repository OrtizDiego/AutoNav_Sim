# Gazebo models

Models the worlds load with `model://<name>`. `sim.launch.py` adds the
installed `share/my_bot/models` to `GAZEBO_MODEL_PATH`, so Gazebo finds them
without its online model database.

## iscas_museum

The museum in `room.world`, `ball.world`, `person.world` and
`two_person.world` (and the one `maps/my_map` was built in).

- Author: Changkun Chai (ISCAS), see `model.config`
- Source: [osrf/gazebo_models](https://github.com/osrf/gazebo_models),
  added in ad4aa2a (2018), removed in c76b4cc (2023, "Removed more large
  files"); copied unchanged from c76b4cc^.
- License: Creative Commons Attribution 3.0 Unported, the license of
  osrf/gazebo_models (https://creativecommons.org/licenses/by/3.0/).

It is no longer in the repository behind Gazebo's online model database,
so a fresh container or CI runner could not count on downloading it.
