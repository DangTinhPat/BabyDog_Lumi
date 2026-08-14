# control input message
#
# command values:
#   0 = NONE  (hold current state)
#   1 = STAND (interpolate to stand_pos and hold)
#   2 = SIT   (interpolate to sit_pos and hold)
#   9 = ESTOP (torque off, back to passive - motors go limp)
int32 command
