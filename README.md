# degeneracy_corrector

This pkg aims to detect, filter, and correct degenerate odometry messages.

### Use-Case
The non-degenerate odometries can be combined to obtain a better localization
estimate using the robot_localization package.

### I/O
Topics in

  * Bool:     /is_degen
  * Odometry: /odom

Topics out

  * Odometry: /odom

### Nodes
degen_detector

  * Compares the velocities of 3-9 odometry messages and marks outliers as degenerate
  
degen_filter

  * Prevents publishing of an odometrty if marked degenerate.
  * Adds correction factor to odometry preventing drift while degenerate. 
