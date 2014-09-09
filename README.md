strands_movebase
================

A repository for all the STRANDS-augmented movebase, including 3D obstacle avoidance, etc. Relies on scitos_2d_navigation if it is configured to only use laser scan input for navigation, https://github.com/strands-project/scitos_2d_navigation.

# Usage

  * `roslaunch strands_movebase movebase.launch map:=/path/to/map.yaml`
  * To be able to use this package you have to create a map with gmapping. This can be run with `rosrun gmapping slam_gmapping`, save the map with `rosrun map_server map_saver`.
  * Each launch file takes the argument `map` which is the path to the map saved with gmapping.
  * Optionally, provide a `no_go_map`, the path to a map annotated with no-go areas.
  * If you do not want to launch it with the 3d obstacle avoidance, provide the additional argument `with_camera:=false`.

If you run with the camera option, be sure that you have a depth camera publishing on the topic `chest_xtion`, for more details see https://github.com/strands-project/strands_movebase/calibrate_chest.
