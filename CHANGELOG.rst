^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package calibrate_chest
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.0.2 (2014-08-27)
------------------
* added Boost::Thread
* added PCL dependency
* Contributors: Marc Hanheide

0.0.1 (2014-08-21)
------------------
* Removed my error.
* Renamed ros_datacentre to mongodb_store
  This simply bulk replaces all ros_datacentre strings to mongodb_store strings inside files and also in file names.
  Needs `strands-project/ros_datacentre#76 <https://github.com/strands-project/ros_datacentre/issues/76>`_ to me merged first.
* Made call unambiguous. Needs testing.
* Renamed ros_datacentre to mongodb_store
  This simply bulk replaces all ros_datacentre strings to mongodb_store strings inside files and also in file names.
  Needs `strands-project/ros_datacentre#76 <https://github.com/strands-project/ros_datacentre/issues/76>`_ to me merged first.
* Fixed misspelling of ros_datacentre in package.xml
* Still some wrong entries of strands_datacentre
* Merge branch 'hydro-devel' into groovy-devel
  Conflicts:
  calibrate_chest/CMakeLists.txt
  calibrate_chest/package.xml
* Switched strands_datacentre to ros_datacentre in here.
* [calibrate_chest] remove version number
* [calibrate_chest] pcl version
* Added a proper desciption of what the package does
* Fixed a bug where all of the dependencies on the datacentre where not defined properly which made parallell builds fail and filled out package.xml
* Updated the position of the camera
* Got the publisher working now, running it in scitos_state_publisher launch file for ten seconds to update urdf with latest calibration parameters
* Added broadcaster of chest clibration, does not really work
* Added comments and cleanup
* Added storage of calibration parameters in strands datacentre
* The angle in the urdf model seems to be right the other way
* Added chest calibratioN
* Contributors: Christian Dondrup, Nick Hawes, Nils Bore, Rares Ambrus, cburbridge, cdondrup
