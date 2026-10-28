# SI: rad, rad/s, Nm, Nm/rad, Nm.s/rad. Output-side effort, never motor current.
uint8 TORQUE=0
uint8 POSITION_PD=1
string<=64 experiment_id
string<=128 model_id
string<=128 clock_id
uint64 sequence
int64 send_ns
int64 valid_for_ns
string[14] joint_names
uint8 mode
float64[14] position
float64[14] velocity
float64[14] effort
float64[14] kp
float64[14] kd
