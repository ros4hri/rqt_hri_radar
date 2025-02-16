^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rqt_human_radar
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.4.0 (2024-12-09)
------------------
* svg template: escape parantheses in RDF class names
* linting
* environment laoder: ensure we can overwrite the template
* rdf editor: add optional drop down selection for predicates
* update objects classes to DBpedia class names
* add a RDF properties editor, attached to objects
* load environments from SVG
  See Settings > Downlaod template and Settings > Load environment
* sim: locally added object can be externally moved by publishing to /interaction_sim/<id>/move_to
* Contributors: Séverin Lemaignan

2.3.0 (2024-11-27)
------------------
* [cmake] remove headers that do not need MOC processing
* Major refactor, using QGraphicsScene/QGraphicsView
  This removes a lot of code related to zooming/drag&drop, etc
  This refactoring leads to new features, including:
  - fields of view for all agents
  - new 'simulated' human
* Contributors: Séverin Lemaignan

2.2.1 (2024-10-16)
------------------
* less verbose output for failing person transforms
* Contributors: Séverin Lemaignan

2.2.0 (2024-10-15)
------------------
* properly scale the robot and humans icons
* use TIAGo Pro head instead of ARI
* Contributors: Séverin Lemaignan

2.1.0 (2024-10-11)
------------------
* make simulation of objects a setting (off by default)
* do not block 1s when frame is not found
* linting
* add apple and pear as additonal objects, fix OWL classes
* update the knowledge base with seen/unseen objects
* add option to show the robot's field of view + corresponding settings
  While here, minor refactor of the 'show ids' btn
* make sure the robot icon zooms in and out properly
  + update ARI's icon so that sellion_link is ~at origin
* add a zoom slider to the UI
* ensure the dropped objects are accurately placed + follow resize
* broadcast the position of dropped objects as TF frames
* publish to the knowledge base when new objects are added/removed
* add support to delete dropped objects by right-clicking on them
* Add 'clear objects' btn
  This reverts commit 7e5330cc442631595e2d7948a5fbc429958ae7f6.
* WIP to place objects in front of the robot
* import a bunch of icons for various objects
* Contributors: Séverin Lemaignan

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
