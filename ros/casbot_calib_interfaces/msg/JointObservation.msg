# Monotonic timestamps require an identical clock_id before cross-host subtraction.
# sample_ns is acquisition; driver_receive_ns and publish_ns are different events.
string<=64 experiment_id
string<=128 model_id
string<=128 clock_id
# Raw device acquisition may use a different clock; do not silently equate them.
string<=128 sample_clock_id
uint64 sequence
uint64 publish_sequence
int64 sample_ns
int64 driver_receive_ns
int64 publish_ns
bool sample_time_known
string[14] joint_names
float64[14] position
float64[14] velocity
float64[14] current
float64[14] estimated_effort
uint32[14] status
bool watchdog_active
uint64 accepted_commands
uint64 adopted_commands
uint64 rejected_commands
