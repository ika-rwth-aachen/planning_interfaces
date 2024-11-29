# The driveable space describes the area that can be driven over by the vehicle without causing damage to the vehicle.
# It also explicitly includes areas that are not allowed to be driven on according to valid traffic regulations, 
# but which could theoretically be driven on (e.g. an opposing lane).

Boundaries boundaries # Boundaries restricting the driveable space
geometry_msgs/Polygon[] restricted_areas # Additional areas that restrict the driveable space in between the boundaries