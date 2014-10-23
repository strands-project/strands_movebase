^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package strands_movebase
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
