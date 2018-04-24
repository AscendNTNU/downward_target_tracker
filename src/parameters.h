#define DEVICE_NAME        "/dev/video0" 
#define USE_CAMERA_NODE     1 // Should be set to 1 if you want to get camera data from an external ROS node instead
#define USE_SSE             0 // Set to 1 if you want to enable SIMD optimizations

// Topic names
#define IMU_POSE_TOPIC     "/mavros/local_position/pose" // (input) best current estimate of drone pose relative grid
#define TRACKS_TOPIC       "/target_tracker/tracks"      // (output) list of tracked targets
#define IMAGE_TOPIC        "/target_tracker/image"       // (output) compressed camera frame
#define INFO_TOPIC         "/target_tracker/info"        // (output) target bounding boxes, detections, and adjustable parameters
#define LINE_COUNTER_TOPIC "/line_counter/pose"          // (output) line counter grid detection
#define SELECTED_TOPIC     "/target_debug/selected"      // (output) unique_id of the highlighted target
#define CAMERA_TOPIC       "/fisheye_raw/compressed" // (input) image data feed from fisheye camera

// Minimum required seconds between publishing debug info and compressed JPEG frame
#define INFO_PUBLISH_INTERVAL  0.0f
#define IMAGE_PUBLISH_INTERVAL 0.0f


// Camera/IMU calibration (see README_example_calibration.md)
#define CAM_IMU_RX_INIT    (0.0f)
#define CAM_IMU_RY_INIT    (0.0f)
#define CAM_IMU_RZ_INIT    (-3.14f/2.0f)
#define CAM_IMU_TX_INIT    (0.0f)
#define CAM_IMU_TY_INIT    (0.0f)
#define CAM_IMU_TZ_INIT    (0.0f)


// Color classification thresholds (see README.md)
#define R_G_INIT           (3.0f) // Minimum red/green ratio
#define R_B_INIT           (2.0f) // Minimum red/blue ratio
#define R_N_INIT           (3.3f) // Minimum average brightness (r+g+b)/3
#define G_R_INIT           (1.6f) // Minimum green/red ratio
#define G_B_INIT           (1.5f) // Minimum green/blue ratio
#define G_N_INIT           (3.3f) // Minimum average brightness (r+g+b)/3


// Fisheye camera parameters (see README.md)
#define USBCAM_DEBUG       1

// If USE_CAMERA_NODE == 1, these values must be the same as the one specified in the camera node
#define CAMERA_WIDTH       800 
#define CAMERA_HEIGHT      600 

#define CAMERA_BUFFERS     3      // Change if frame rate is lower than 60 Hz (see Timing window in debugger)
#define CAMERA_LEVELS      0      // Downscale factor (0=none, 1=half, 2=quarter)
#define CAMERA_F_INIT      288.0f // Fisheye parameter: Focal length
#define CAMERA_U0_INIT     376.0f // Fisheye parameter: Center X
#define CAMERA_V0_INIT     280.0f // Fisheye parameter: Center Y


// Line counter parameters
#define RUN_LINE_COUNTER           1
#define WHITE_THRESHOLD_R_INIT     (173.0f)
#define WHITE_THRESHOLD_G_INIT     (197.0f)
#define WHITE_THRESHOLD_B_INIT     (161.0f)
#define WHITE_THRESHOLD_D_INIT     (41.0f)
#define PINHOLE_FOV_X_INIT         (137.0f * 3.14f/180.0f)
#define SOBEL_THRESHOLD_INIT       (10)
#define MAXIMA_THRESHOLD_INIT      (10)
#define MAX_ERROR_INIT             (0.5f)
#define TILE_WIDTH_INIT            (1.0f)
#define CAMERA_LEVELS_LINE_COUNTER (0) // Downscale factor (0=none, 1=half, 2=quarter)
