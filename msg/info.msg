float32 dt_frame
float32 dt_jpeg_to_rgb
float32 dt_track_targets
float32 dt_cycle

float32 camera_f
float32 camera_u0
float32 camera_v0
int32 camera_w
int32 camera_h
int32 image_w
int32 image_h

# R_cam^imu
float32 cam_imu_rx
float32 cam_imu_ry
float32 cam_imu_rz

# T_cam/imu^imu
float32 cam_imu_tx
float32 cam_imu_ty
float32 cam_imu_tz

float32 r_g
float32 r_b
float32 r_n
float32 g_r
float32 g_b
float32 g_n

# R_imu^world
float32 imu_rx
float32 imu_ry
float32 imu_rz

# T_imu/world^world
float32 imu_tx
float32 imu_ty
float32 imu_tz

#
# debug info for tracks
#

# image coordinates are specified inside this resolution
int32 image_x
int32 image_y

# last seen detection for target[i]
float32[] last_seen_u
float32[] last_seen_v
float32[] last_seen_u1
float32[] last_seen_v1
float32[] last_seen_u2
float32[] last_seen_v2
float32[] last_seen_x
float32[] last_seen_y
float32[] last_seen_t

# all raw detections
int32     num_detections
float32[] detection_u
float32[] detection_v
float32[] detection_u1
float32[] detection_v1
float32[] detection_u2
float32[] detection_v2
