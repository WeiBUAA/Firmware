# VEHICLE_VTOL_STATE, should match 1:1 MAVLinks's MAV_VTOL_STATE
uint8 VEHICLE_VTOL_STATE_UNDEFINED = 0
uint8 VEHICLE_VTOL_STATE_TRANSITION_TO_FW = 1
uint8 VEHICLE_VTOL_STATE_TRANSITION_TO_MC = 2
uint8 VEHICLE_VTOL_STATE_MC = 3
uint8 VEHICLE_VTOL_STATE_FW = 4



uint64 timestamp			# time since system start (microseconds)
uint64 vehicle_sysidt_state             # State machine record for system identification experiment

uint64  ticks_since_trans               # for ILC debug
float32 pitch_sp
float32 pitch_ang
float32 vert_acc_cmd
float32 thrust_cmd
float32 bx_acc_cmd
float32 bx_acc_err
float32 bx_acc_i              # integrator for body-x acc controller
float32 acc_iz_fdb
float32 acc_bx_filt
float32 acc_bz_filt
float32 delt_thrust
float32 rollrot
float32 lat_dist
float32 pitchrot
float32 longitudinal_dist
float32 lateral_v
float32 longitudinal_v
float32 vz_cmd
float32 vy_cmd
float32 vx_cmd
float32 global_counter

bool vtol_in_rw_mode			# true: vtol vehicle is in rotating wing mode
bool vtol_in_trans_mode
bool in_transition_to_fw		# True if VTOL is doing a transition from MC to FW
bool vtol_transition_failsafe		# vtol in transition failsafe mode
bool fw_permanent_stab			# In fw mode stabilize attitude even if in manual mode