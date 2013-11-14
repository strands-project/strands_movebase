scitos_2d_navigation
====================

The scitos_2d_navigation stack holds common configuration options for running the 2D navigation stack on a Scitos G5 robot.

# Usage

  * To be able to use this package you have to create a map with gmapping. This can be run with `rosrun gmapping slam_gmapping`, save the map with `rosrun map_server map_saver`.
  * Each launch file takes the argument `map` which is the path to the map saved with gmapping.
  * To just launch the DWA planner together with AMCL localization do `roslaunch scitos_2d_navigation amcl.launch map:=/path/to/map.yaml`.
  * If you want to launch it with the 3d obstacle avoidance, provide the additional argument `with_camera:=true`. Make sure that you have a depth camera publishing on the topic `chest_xtion`, for more details see https://github.com/strands-project/scitos_common.
