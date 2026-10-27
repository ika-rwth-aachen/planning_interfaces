# Ordered list of WGS84 waypoints, including the final destination
geographic_msgs/GeoPoint[] waypoints

# Wait time per waypoint in seconds:
# -1: Intermediate destination (no stop)
# >=0: Destination with waiting time
float32[] wait_times
