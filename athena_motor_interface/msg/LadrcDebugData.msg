# LADRC internal state and control variables
float32 v_ref
float32 p_hold
bool is_position_hold

# Read motor position from filter
float32 pos_meas

# Estimated states from ESO
float32 x1_hat
float32 x2_hat
float32 x3_hat

# Torques
float32 tau_raw
float32 tau_ff
float32 tau_aug
float32 tau_safe
float32 output

# Slip detection state
int32 slip_holdoff_counter
float32 dt
