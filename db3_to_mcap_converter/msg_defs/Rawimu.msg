# Reconstructed starneto_gps_msgs/Rawimu definition
# Validated against actual byte layout in the source bag.
# Mirrors the NovAtel RAWIMU log convention: z_accel, -y_accel, x_accel, z_gyro, -y_gyro, x_gyro
Header header

NovatelMessageHeader novatel_msg_header

uint32 gps_week
float64 gps_seconds
uint32 imu_status

# Accelerations - NovAtel convention z, -y, x (m/s^2 after scaling)
float64 z_accel
float64 y_accel
float64 x_accel

# Angular rates - NovAtel convention z, -y, x (rad/s after scaling)
float64 z_gyro
float64 y_gyro
float64 x_gyro
