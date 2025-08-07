std_msgs/Header header

geometry_msgs/Point destination                 # destination (potentially mapped to suggested lane reference line)
geometry_msgs/Point[] intermediate_destinations # route should be planned over those points (could be empty) 

RouteElement[] route_elements                   # cross sections of drivable adjacent lanes in driving direction towards destination

uint64 starting_route_element_idx               # index of the route element right before the starting point
uint64 current_route_element_idx                # index of the first route element of the remaining route, right before the current position
uint64 destination_route_element_idx            # index of the route element right behind the destination
