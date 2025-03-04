uint8 DOMAIN_STREET=0
uint8 DOMAIN_INTERSECTION=1
uint8 DOMAIN_ROUNDABOUT=2
uint8 DOMAIN_PARKING=3

uint8 domain_id                             # domain id (if this is good accessable in lanelet2)
LaneElement[] lane_elements                 # lanes in own driving direction
uint8 current_lane_id                       # lane id of shortest path ("ego lane")
float64 current_s                           # distance along route
bool lane_change                            # indicates if shortest path is in lane change mode (could also be done by making current_lane a vector)

geometry_msgs/Point drivable_space_left     # point of left drivable space bounds
geometry_msgs/Point drivable_space_right    # point of right drivable space bounds