###########################################################################################################################################################
#   REFERENCE Trajectory (independent of vehicle model)
###########################################################################################################################################################
uint8 TYPE_ID=0
uint8 STATE_DIM=4

#   sample points of REFERENCE trajectories have the following entries
#   state information for each sample point:
uint8 T=0                           # time to reach sample point [s]
uint8 X=1                           # x position of sample point in trajectory header frame [m]
uint8 Y=2                           # y position of sample point in trajectory header frame [m]
uint8 V=3                           # absolute velocity in vehicle frame [m/s]
###########################################################################################################################################################
