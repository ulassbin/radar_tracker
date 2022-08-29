uint64 id
int8 INIT_FLAG_ASSIGN = 0
int8 INIT_FLAG_NEW_OBS = 1
int8 init_flag
uint64 update_count # Number of measurement updates done
uint64 prediction_count # Numbr of predictions done
geometry_msgs/Twist velocity
float64 posx
float64 posy
float64 width
float64 height
float64 depth
uint64[] meas_index