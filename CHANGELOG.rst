^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rqt_human_radar
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2024-10-10)
------------------
* remove 'clear objects' btn, as not needed in basic 'radar' mode
* minor linting
* display persons instead of bodies
  This ensures that both bodies and faces will be displayed
* redesigned UI
* ROS 2 linting
* port to ROS2
  while here, change license from BSD to Apache 2.0
* Contributors: Séverin Lemaignan

0.3.0 (2023-05-11)
------------------
* Merge branch 'change-perspective' into 'main'
  reference frame selection
  See merge request ros4hri/rqt_human_radar!2
* defined constants as const
* reference frame selection
  it's now possible deciding which reference frame to use
* adding build dependency on hri
* [wip] defining structures for changing perspective
* Contributors: lorenzoferrini

0.2.1 (2022-12-13)
------------------
* changing variable names to overcome shadowing errors
* fixed CMakeLists.txt to overcome shadowing issues
* adding missing build dependencies
  The package was previously missing a series of build dependencies:
  - tf
  - hri
  - qt5base
  - gt5svg5
* Contributors: lorenzoferrini

0.2.0 (2022-10-18)
------------------

* {rqt_engagement_radar->rqt_human_radar}
* add BSD license
* removed scripts folder installation
  since scripts folder was not used in the end, removed any reference
  to its installation in the CMakelists.txt
* removed .png person icon (now using .svg)
* Using body position for people placing.
  No more using face for icon placing in the radar canvas.
  Commented the code to make it more understandable.
* removed unused tick boxes
* removed leftover code
* fixed graphics and resizing
* distance information displayed
  it is now possible displaying the distance information by clicking
  on a person icon
* added pixels-per-meter spinbox
  in setting it is now possible to set the pixels per meter.
  Spinbox info:
  - minimum = 50
  - maximum = 600
  - single step = 10
* removed any reference to attention/fov cones
* svg object management
  - size of the polygon containing the svg based on the svg size
  - defined constants for the rendering size
  - removed references to the .png person image
  - introduced a check on right loading of the svg image
* ranges and agles info visualization
  adding information about the distance each range represents and
  angles visualization
* [wip] rendering person icon from svg
* revisited range painting process
* [WiP] display of person info
  display of person info when hovering with mouse over the image
  of a person
* Fixed person image rotation process
* Multitab version plugin - first version
  First, semi-mockup version of the multitab version of the rada
  plugin. Two different tabs: one for the radar itself, one for the
  settings.
* add BSD license
* Contributors: Séverin Lemaignan, lorenzoferrini

0.1.0 (2022-09-12)
------------------
* Initial release: display a top-down 'radar' view of the humans detected around
  the robot
* Contributors: Lorenzo Ferrini, Séverin Lemaignan
