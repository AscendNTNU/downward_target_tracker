// Debugging parameters for author: Set to 0 during testing with drone
#define TESTING_WITH_LAPTOP    0 // 1 - use author's laptop camera instead of fisheye
#define DUMMY_IMAGE            1 // 1 - use a static image instead of usb camera input
#define DEVICE_NAME            "/dev/video0"

// Topic on which pose of drone relative grid is published
#define IMU_POSE_TOPIC "/mavros/vision_pose/pose"

// Minimum required seconds between publishing debug info
// Used in debugger to visualize bounding boxes and raw detections
// Set to zero to publish every frame
#define INFO_PUBLISH_INTERVAL  0.0f

// Minimum required seconds between publishing compressed JPEG frame
// Set to zero to publish every frame
// If network bandwidth is too slow, you might want to increase this
#define IMAGE_PUBLISH_INTERVAL 0.0f

// Camera rotation offset (camera frame in imu frame)
// Mathematically:  (R_cam^imu = Rz(cam_imu_rz)Ry(cam_imu_ry)Rx(cam_imu_rx))
#define CAM_IMU_RX_INIT    0.0f
#define CAM_IMU_RY_INIT    0.0f
#define CAM_IMU_RZ_INIT    0.0f

// Camera position offset (camera center relative imu center in imu frame)
// Mathematically: (T_cam/imu^imu = {cam_imu_tx, cam_imu_ty, cam_imu_tz})
#define CAM_IMU_TX_INIT    0.0f
#define CAM_IMU_TY_INIT    0.0f
#define CAM_IMU_TZ_INIT    0.0f

// Red classification thresholds
#define R_G_INIT           3.0f       // minimum red/green ratio
#define R_B_INIT           2.0f       // minimum red/blue ratio
#define R_N_INIT           10.0f/3.0f // minimum average brightness (r+g+b)/3

// Green classification thresholds
#define G_R_INIT           1.6f       // minimum green/red ratio
#define G_B_INIT           1.5f       // minimum green/blue ratio
#define G_N_INIT           10.0f/3.0f // minimum average brightness (r+g+b)/3

// Grid detector parameters
#define WHITE_THRESHOLD_R_INIT (222.0f)
#define WHITE_THRESHOLD_G_INIT (222.0f)
#define WHITE_THRESHOLD_B_INIT (222.0f)
#define WHITE_THRESHOLD_D_INIT (70.0f)
#define PINHOLE_FOV_X_INIT     (137.0f * 3.14f/180.0f)
#define SOBEL_THRESHOLD_INIT   (10)
#define MAXIMA_THRESHOLD_INIT  (10)
#define MAX_ERROR_INIT         (0.5f)
#define TILE_WIDTH_INIT        (1.0f)

// Fisheye camera parameters
#if DUMMY_IMAGE==0 && TESTING_WITH_LAPTOP==0
#define USBCAM_DEBUG       1
#define CAMERA_WIDTH       800
#define CAMERA_HEIGHT      600
#define CAMERA_BUFFERS     3      // Might want to change if output rate (see Timing window in debugger) is lower than 60 Hz
#define CAMERA_LEVELS      2      // Downscale factor (0=none, 1=half, 2=quarter)
#define CAMERA_F_INIT      434.0f // Fisheye parameter: 'Focal' length
#define CAMERA_U0_INIT     400.0f // Fisheye parameter: Center X
#define CAMERA_V0_INIT     300.0f // Fisheye parameter: Center Y
#endif

//
// Ignore these:
//

// Author's laptop camera parameters
#if TESTING_WITH_LAPTOP==1
#define USBCAM_DEBUG       1
#define CAMERA_WIDTH       800
#define CAMERA_HEIGHT      600
#define CAMERA_BUFFERS     3
#define CAMERA_LEVELS      2 // Downscale factor (0=none, 1=half, 2=quarter)
#define CAMERA_F_INIT      434.0f
#define CAMERA_U0_INIT     375.0f
#define CAMERA_V0_INIT     275.0f
#endif

// Static image camera parameters
#if DUMMY_IMAGE==1
#define CAMERA_WIDTH       1280
#define CAMERA_HEIGHT      720
#define CAMERA_LEVELS      3
#define CAMERA_F_INIT      494.0f
#define CAMERA_U0_INIT     649.0f
#define CAMERA_V0_INIT     335.0f
#endif
