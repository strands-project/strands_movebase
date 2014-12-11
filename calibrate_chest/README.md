calibrate_chest
===============

The new way to use this package is through the `calibration_server` action server. To use it, run `rosrun calibrate_chest calibration_server` and, in another terminal, `rosrun actionlib axclient.py /calibrate_chest`. To compute a new transformation between the camera and the floor, enter command `calibrate`. If you want to publish an already saved calibration (the calibration is saved in mongodb between runs), enter command `publish`. When you run the calibrate command, it will check that you are closer than 3 degrees from the desired 46 degrees angle of the chest camera. Otherwise, no calibration will be stored.

# calibrate_chest node (legacy)

  * Do `rosrun calibrate_chest calibrate_chest` with the datacentre running if you want to store the parameters there, make sure that the largest visible plane for the chest camera is the floor. Also, notice that you have to have the chest camera running and publishing on topic `chest_xtion`.
  * To use these parameters when you launch the bringup next time, be sure to have the datacentre running.
  * The urdf can be updated manually by doing `rosrun calibrate_chest chest_calibration_publisher` if you don't want to restart the larger system (e.g. `strands_movebase.launch`, which includes this).

# Example

When running `rosrun calibrate_chest calibrate_chest` the node tries to find the largest visible plane and determine the angle and height of the chest camera. It will display a window of the current point cloud, with the points belonging to the floor coloured in red. It should look something like the following, you might need a mouse to rotate:

![Calibration point cloud example](https://github.com/strands-project/strands_movebase/tree/hydro-devel/calibrate_chest/data/chest.png "Example of how the calibration should look.")

Close the window to save the calibration.
