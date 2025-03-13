uint8 UNKNOWN=0
uint8 YIELD=1
uint8 STOP=2
uint8 TRAFFIC_LIGHT=3
uint8 RIGHT_OF_WAY=4
uint8 SPEED_LIMIT=5

uint8 META_VALUE_STATE_UNKNOWN=0

# meta values for YIELD, STOP, TRAFFIC_LIGHT
uint8 META_VALUE_MOVEMENT_ALLOWED=1     # movement is allowed
uint8 META_VALUE_MOVEMENT_RESTRICTED=2  # movement is restricted

# meta values for SPEED_LIMIT
uint8 META_VALUE_SPEED_1=1              # speed limit of 1km/h
uint8 META_VALUE_SPEED_30=30            # speed limit of 30km/h
uint8 META_VALUE_SPEED_50=50            # speed limit of 50km/h
uint8 META_VALUE_SPEED_70=70            # speed limit of 70km/h

uint8 type
uint8 meta_value