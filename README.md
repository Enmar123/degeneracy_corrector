degeneracy_corrector

This pkg aims to provide a map->odom transform that will correct the positioning of a 
degenerate odom->base_link transform.

The node takes in the bool topic /is_degenerate as well as the /odom topic to be corrected 
and the target /odom.
Upon recieving true for /is_degenerate the current and target /odom is evaluated. The node
then creates a /tf topic from map_frame to odom_frame that shifts the degenerate/odom to
the target/odom relative to the map_frame.

Topics in

  * /is_degenerate
  * /odom
  * /odom 

Topics out

  * /tf
