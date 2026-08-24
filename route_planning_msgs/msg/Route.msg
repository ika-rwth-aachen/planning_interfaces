std_msgs/Header header


geometry_msgs/Point start                               # starting point (potentially mapped to suggested lane reference line)
geometry_msgs/Point destination                         # destination (potentially mapped to suggested lane reference line)
geometry_msgs/Point[] intermediate_destinations         # route should be planned over those points (could be empty)

bool has_route_elements                                 # whether route contains (enriched) route elements or only a reference line
RouteElement[] route_elements                           # cross sections of drivable adjacent lanes in driving direction towards destination (if has_route_elements)
uint64 start_route_element_idx                          # index of the route element right before the starting point
uint64 current_route_element_idx                        # index of the first route element of the remaining route, right before the current position
uint64 destination_route_element_idx                    # index of the route element right behind the destination
uint64 INVALID_ROUTE_ELEMENT_IDX=18446744073709551615   # uint64_max

Point2d32[] reference_line_deltas                       # reference line point deltas relative to previous point (if !has_route_elements)
