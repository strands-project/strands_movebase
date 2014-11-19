calibrate_chest
===============

  * Do `rosrun calibrate_chest calibrate_chest` with the datacentre running if you want to store the parameters there, make sure that the largest visible plane for the chest camera is the floor. Also, notice that you have to have the chest camera running and publishing on topic `chest_xtion`.
  * To use these parameters when you launch the bringup next time, be sure to have the datacentre running.
  * The urdf can be updated manually by doing `rosrun calibrate_chest chest_calibration_publisher` if you don't want to restart the larger system (e.g. `strands_movebase.launch`, which includes this).

# calibrate_chest

When running `rosrun calibrate_chest calibrate_chest` the node tries to find the largest visible plane and determine the angle and height of the chest camera. It will display a window of the current point cloud, with the points belonging to the floor coloured in red. It should look something like the following, you might need a mouse to rotate:
![Calibration point cloud example](https://github.com/strands-project/strands_movebase/tree/hydro-devel/calibrate_chest/data/chest.png "Example of how the calibration should look.")
