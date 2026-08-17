# control input message
#
# command values:
#   0 = NONE  (hold current state)
#   1 = STAND (interpolate feet in Cartesian space, solve IK, and hold)
#   2 = SIT   (interpolate feet in Cartesian space, solve IK, and hold)
#   9 = ESTOP (torque off, back to passive - motors go limp)
int32 command
