float32 dt_frame
float32 dt_jpeg_to_rgb
float32 dt_track_targets
float32 dt_cycle

#### CAMERA INTRINSICS
float32 camera_f
float32 camera_u0
float32 camera_v0
int32 camera_w
int32 camera_h

#### CAMERA EXTRINSICS
float32 cam_imu_rx
float32 cam_imu_ry
float32 cam_imu_rz
float32 cam_imu_tx
float32 cam_imu_ty
float32 cam_imu_tz

#### COLOR THRESHOLDS
float32 r_g
float32 r_b
float32 r_n
float32 g_r
float32 g_b
float32 g_n

#### LAST RECEIVED DRONE POSE
float32 imu_rx
float32 imu_ry
float32 imu_rz
float32 imu_tx
float32 imu_ty
float32 imu_tz

#### TRACKS
int32     num_targets
int32[]   unique_id
float32[] position_x
float32[] position_y
float32[] velocity_x
float32[] velocity_y
float32[] detection_rate
#### (u,v) COORDINATE RESOLUTION
int32 image_x
int32 image_y
#### LAST SEEN DETECTION FOR target[i]
float32[] last_seen_u
float32[] last_seen_v
float32[] last_seen_u1
float32[] last_seen_v1
float32[] last_seen_u2
float32[] last_seen_v2
float32[] last_seen_x
float32[] last_seen_y
float32[] last_seen_t
#### DETECTIONS
int32     num_detections
float32[] detection_u
float32[] detection_v
float32[] detection_u1
float32[] detection_v1
float32[] detection_u2
float32[] detection_v2
