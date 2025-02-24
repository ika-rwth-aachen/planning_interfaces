std_msgs/Header header

geometry_msgs/Point destination # route destination (x,y,z)

uint8 current_speed_limit       # speed limit at current position [km/h]

RouteElement[] traveled_route
RouteElement[] remaining_route