^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package strands_movebase
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#13 <https://github.com/strands-project/strands_movebase/issues/13>`_ from nilsbore/backtrack
  [strands_movebase] Add option to use head_camera as well
* Changed the unknown_threshold to 9 as well for consistency
* Removed commented code
* Removed unnecessary code and switched to the topics used in backtrack
* Fixed the parameter settings, it actually works
* Tidied up a bit and changed to the correct topic
* Forgot to change the height
* Added the option to have an extra head camera, with a higher voxelgrid
* Contributors: Marc Hanheide, Nils Bore, Rares Ambrus

0.0.7 (2014-11-09)
------------------
* final and tested version of loader
* new machine tags
* Fixed typos in launch files
* Added launch file options for changing the obstacle and stair heights to enable to tune the robustness
* Contributors: Jaime Pulido Fentanes, Nils Bore, Rares Ambrus

0.0.6 (2014-10-27)
------------------
* Decreased the cost_scaling_factor, making the robot stay away from walls if possible. This made navigation more robust on Rosie, particularly through doors
* Contributors: Nils Bore

0.0.5 (2014-10-23)
------------------
* added changelogs
* Added launching of chest transform state publisher together with 3d movebase
* Added the dependencies in catkin_package
* Placed include files in include/strands_movebase and added install targets
* Modified config.launch to use strands_movebase nodes and configs instead of scitos_2d_navigation
* Renamed the launch files and made 3d obstacle avoidance the default
* Corrected the homepage
* Moved the headers to include folder
* Removed move_base.launch since that will be in scitos_2d_navigation
* Mad strands_movebase a package within the repo, to be able to put e.g. chest_calibration in another package
* Contributors: Marc Hanheide, Nils Bore
