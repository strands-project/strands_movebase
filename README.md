strands_movebase
================

A repository for all the STRANDS-augmented movebase, including 3D obstacle avoidance, etc. Relies on scitos_2d_navigation if it is configured to only use laser scan input for navigation, https://github.com/strands-project/scitos_2d_navigation.

# Usage

  * `roslaunch strands_movebase movebase.launch map:=/path/to/map.yaml`
  * To be able to use this package you have to create a map with gmapping. This can be run with `rosrun gmapping slam_gmapping`, save the map with `rosrun map_server map_saver`.
  * Each launch file takes the argument `map` which is the path to the map saved with gmapping.
  * Optionally, provide a `with_no_go_map:=true` and `no_go_map`, the path to a map annotated with no-go areas.
  * If you do not want to launch it with the 3d obstacle avoidance, provide the additional argument `with_camera:=false`.
  * Provide `camera:=<camera_namespace>` if you have an OpenNI camera publishing on another namespace than the default `chest_xtion`.
  * Optionally provide `z_obstacle_threshold:=<value>`, where `<value>` m is the distance from the floor above which we consider points as obstacles. Making this larger than the default 0.1 improves navigation robustness but may lead to missing small obstacles.
  * Same goes for `z_stair_threshold:=<value>`, the distance below which points are considered negative obstacles. Again, increasing improves robustness, but make sure you don't have any stairs with smaller gaps than this.

If you run with the camera option, be sure that you have a depth camera publishing on the `camera_namespace` topic. The camera also needs a valid TF transform connecting it to `base_link`. For more details on the Strands solution, see https://github.com/strands-project/strands_movebase/tree/hydro-devel/calibrate_chest and https://github.com/strands-project/strands_movebase/tree/hydro-devel/strands_description.
