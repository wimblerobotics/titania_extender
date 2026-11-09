# Periodic status for one extender motor.
std_msgs/Header header

string name               # "rotate" (M1) or "extend" (M2)

int32 encoder             # encoder counts
uint8 encoder_status      # RoboClaw encoder status byte
int32 speed               # measured speed, quad pulses per second
float32 current           # motor current, amps
float32 temperature       # RoboClaw board temperature, degrees C
float32 main_battery_voltage
uint32 error_status       # Low 16 bits: errors; high 16 bits: warnings
string error_string       # decoded errors and warnings

int32 target_position     # last commanded target, encoder counts
bool move_active          # true while an action goal is executing

# Limit switches (dummy until hardware is installed; always false).
bool min_limit_active
bool max_limit_active
