builtin_interfaces/Time stamp

# Producer states (lowercase canonical strings)
string navigation_status        # idle | navigating | stopped | halted | paused | resumed | unknown
string tts_status               # idle | speaking | unknown
string stt_status               # idle | listening | transcribing | unknown
string llm_status               # idle | thinking | responding | unknown
string system_status            # ok | warning | critical | unknown

# Location & motion
string current_location         # nearest named location, or "unknown"
float32 linear_velocity
float32 angular_velocity
float32 pose_x
float32 pose_y
float32 pose_yaw

# Energy
float32 battery_voltage
float32 battery_percentage

# Compute
float32 pi_cpu_percent
float32 pi_temp_c
float32 pi_memory_usage
float32 jetson_cpu_percent
float32 jetson_gpu_percent
float32 jetson_temp_c
float32 jetson_memory_usage
float32 jetson_memory_used_gb
float32 jetson_memory_total_gb

# Human-readable summary
string status_message
