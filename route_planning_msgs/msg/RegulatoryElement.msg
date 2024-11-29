uint8 TYPE_UNKNOWN=0
uint8 TYPE_YIELD=1
uint8 TYPE_STOP=2
uint8 TYPE_TRAFFIC_LIGHT=3
uint8 TYPE_RIGHT_OF_WAY=4
uint8 TYPE_SPEED_LIMIT=5
uint8 type # Type of the regulatory element

uint8 STATE_UNKNOWN=0 # Unknown value
uint8 MOVEMENT_ALLOWED=1 # Movement allowed for stop and yield-signs as well as traffic lights
uint8 MOVEMENT_RESTRICTED=2 # Movement restricted for stop and yield-signs as well as traffic lights
uint8 SPEED_30=30 # Speed value [km/h] for speed limit elements
uint8 SPEED_50=50 # Speed value [km/h] for speed limit elements
uint8 SPEED_70=70 # Speed value [km/h] for speed limit elements
uint8 value # Signal value of the regulatory element (dependant on type)

geometry_msgs/Point[2] effect_line # effect line of the respective regulatory element, restricting the movement (stop, yield, traffic-light) or indicating the validity of a speed limit; z>=0 indicates the s-coordinate of the ref-path of effect-line-intersection (z=-1, if not part of ref-path)
geometry_msgs/Point[] signal_positions # position(s) of the actual signal (e.g. position of the traffic-light or sign)