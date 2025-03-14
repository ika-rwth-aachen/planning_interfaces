std_msgs/Header header

geometry_msgs/Point destination         # destination

RouteElement[] traveled_route_elements  # cross sections of traveled drivable adjacent lanes in driving direction towards destination
RouteElement[] remaining_route_elements # cross sections of remaining drivable adjacent lanes in driving direction towards destination