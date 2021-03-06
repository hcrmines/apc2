# apc2

aliases:
* `baxnet` run avahi
* `baxter` run baxter.sh
* `baxenable` enable baxter
* `baxreset` reset baxter
* `baxjoint` run joint server
* `moveit` launch moveit with kinect
* `kinect` launch freenect
* `xyz_right` or `xyzr` get position of right hand
* `xyz_left` or `xyzl` get position of left hand
* `grippers` calibrate grippers (setup)
* `open_right` open right grippers
* `close_right` close right grippers
* `open_left` open left grippers
* `close_left` close left grippers

nodes:
* `rosrun apc2 save` save a background
* `rosrun apc2 find` find clusters
* `rosrun apc2 pick` pick a cluster to grab
* `rosrun apc2 move` coordinate with moveit

after `rosrun apc2 move`:
* `move 0.3 0.1 0.5` move to x, y, z (arm positioned forward)
* `drop 0.0 0.3 0.1` move to x, y, z (arm position down)

