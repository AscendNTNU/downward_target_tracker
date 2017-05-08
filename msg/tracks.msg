int32 num_targets
int32[] unique_id
float32[] x_hat
float32[] y_hat
float32[] dx_hat
float32[] dy_hat

#
# debug info
#

# image coordinates are specified inside this resolution
int32 image_x
int32 image_y

# last seen detection for target[i]
float32[] u
float32[] v
float32[] u1
float32[] v1
float32[] u2
float32[] v2
float32[] x
float32[] y
float32[] t

# image-space estimated position for target[i]
float32[] u_hat
float32[] v_hat
int32[] confidence

# all raw detections
int32 num_detections
float32[] detection_u
float32[] detection_v
float32[] detection_u1
float32[] detection_v1
float32[] detection_u2
float32[] detection_v2
