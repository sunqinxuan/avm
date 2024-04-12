
Header header

uint64 frame_unmber
int8 fusion_level
int8 status  #4:固定解，5：浮点解，2：伪距差分，1：单点解
#rotation
float64 roll      # x,R, car body (FLU)
float64 pitch      # y,R, car body (FLU)
float64 yaw      # z,R, car body (FLU) utm (0,360)  0--east, 90--north
float32 roll_std	
float32 pitch_std
float32 yaw_std 

float64 qw          # w
float64 qx          # x
float64 qy          # y
float64 qz          # z

float64 angular_velocity_x	
float64 angular_velocity_y
float64 angular_velocity_z 


#position
float64 latitude	
float64 longitude
float64 altitude
float32 latitude_std	
float32 longitude_std
float32 altitude_std

float64 position_x  # utm_east
float64 position_y  # utm_north
float64 position_z  # utm_up
float32 position_x_std	
float32 position_y_std
float32 position_z_std

float64 velocity_x  # utm_east_speed
float64 velocity_y  # utm_north_speed
float64 velocity_z  # utm_up_speed
float32 velocity_x_std	
float32 velocity_y_std
float32 velocity_z_std

float64 velocity_rx  # ins_x_speed
float64 velocity_ry  # ins_y_speed
float64 velocity_rz  # ins_z_speed
float32 velocity_rx_std	
float32 velocity_ry_std
float32 velocity_rz_std

float64 accel_x     # R, car body (RFU)
float64 accel_y     # F, car body (RFU)
float64 accel_z     # U, car body (RFU)



