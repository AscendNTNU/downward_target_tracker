#if RUN_LINE_COUNTER != 1
void line_counter_init() { }
void line_counter_copy(unsigned char *jpg_data, unsigned int jpg_size, float *dt_memcpy) { *dt_memcpy = 0.0f; }

#else
#include <pthread.h>
// #define ASC_GRID_DETECTOR_IMPLEMENTATION
// #define ASC_GRID_DETECTOR_SSE
// #include "asc_grid_detector.h"

// These need to be volatile, otherwise the main_line_counter thread did
// not work properly when I compile with optimizations (-O2).
volatile bool         line_counter_jpg_available = false;
volatile bool         line_counter_using_jpg = false;
volatile unsigned int line_counter_jpg_size;
static unsigned char  line_counter_jpg_data[CAMERA_WIDTH*CAMERA_HEIGHT*3];

void line_counter_copy(unsigned char *jpg_data, unsigned int jpg_size, float *out_dt_memcpy)
{
    float dt_memcpy = 0.0f;
    if (!line_counter_using_jpg)
    {
        uint64_t t1 = getnsec();
        memcpy(line_counter_jpg_data, jpg_data, jpg_size);
        line_counter_jpg_size = jpg_size;
        line_counter_jpg_available = true;
        uint64_t t2 = getnsec();
        dt_memcpy = (t2-t1)/1e9;
    }
    *out_dt_memcpy = dt_memcpy;
}

void *line_counter_main(void *);

void line_counter_init()
{
    pthread_t t;
    pthread_create(&t, NULL, line_counter_main, NULL);
}

#if 1
// For testing: just write shared image to file ("test.jpg")
void *line_counter_main(void *)
{
    printf("line counter!\n");
    for (int frame = 0;; frame++)
    {
        while (!line_counter_jpg_available)
        {
        }
        line_counter_jpg_available = false;
        line_counter_using_jpg = true;

        float dt_fwrite = 0.0f;
        {
            uint64_t t1 = getnsec();
            FILE *f = fopen("test.jpg", "w+");
            assert(f);
            fwrite(line_counter_jpg_data, line_counter_jpg_size, 1, f);
            fclose(f);
            uint64_t t2 = getnsec();
            dt_fwrite = (t2-t1)/1e9;
        }

        usleep(50*1000);

        printf("%d. %.2f ms\n", frame, 1000.0f*dt_fwrite);

        line_counter_using_jpg = false;
    }
}
#else
void *main_line_counter(void *)
{
    printf("[line_counter] Running\n");
    pub = node.advertise<ascend_msgs::LineCounter>("/line_counter/pose", 1);

    const int Ix = CAMERA_WIDTH>>CAMERA_LEVELS_GRID_DETECTOR;
    const int Iy = CAMERA_HEIGHT>>CAMERA_LEVELS_GRID_DETECTOR;
    static unsigned char I_rgb[CAMERA_WIDTH*CAMERA_HEIGHT*3];
    static unsigned char I_gray[CAMERA_WIDTH*CAMERA_HEIGHT];
    for (int frame = 0;; frame++)
    {
        // WAIT UNTIL MAIN THREAD RECEIVED FRAME FROM CAMERA
        while (!line_counter_jpg_available)
        {
        }

        // ACQUIRE LOCK ON FRAME
        line_counter_jpg_available = false; // we have now 'consumed' this frame. main thread will set this to true.
        line_counter_using_jpg = true; // main thread cannot update 'available' until this is set to false.

        // CONVERT JPEG TO RGB
        float dt_jpeg_to_rgb = 0.0f;
        {
            uint64_t t1 = getnsec();
            if (!usbcam_jpeg_to_rgb(Ix, Iy, I_rgb, jpg_data, jpg_size))
            {
                printf("[line_counter] Failed to decompress JPEG\n");
                line_counter_using_jpg = false; // unlock
                continue;
            }
            uint64_t t2 = getnsec();
            dt_jpeg_to_rgb = (t2-t1)/1e9;
        }

        // CONVERT RGB IMAGE TO GRAYSCALE IMAGE (EXTRACTING 'WHITE' PIXELS)
        float dt_threshold = 0.0f;
        {
            uint64_t t1 = getnsec();
            asci_threshold(I_rgb, I_gray, Ix, Iy,
                           white_threshold_r,
                           white_threshold_g,
                           white_threshold_b,
                           white_threshold_d);
            uint64_t t2 = getnsec();
            dt_threshold = (t2-t1)/1e9;
        }

        // COMPUTE CAMERA POSE
        // (rotation relative grid, and height relative grid in grid coordinates)
        float cam_ry, cam_rx, cam_z;
        {
            mat3 imu_rot = m_rotz(imu_rz)*m_roty(imu_ry)*m_rotx(imu_rx);
            vec3 imu_pos = m_vec3(imu_tx, imu_ty, imu_tz);
            mat3 cam_imu_rot = m_rotz(cam_imu_rz)*m_roty(cam_imu_ry)*m_rotx(cam_imu_rx);
            vec3 cam_imu_pos = m_vec3(cam_imu_tx, cam_imu_ty, cam_imu_tz);
            mat3 rot = imu_rot*cam_imu_rot;
            vec3 pos = imu_pos + imu_rot*cam_imu_pos;

            cam_z = pos.z;
            float cam_rx,cam_ry,cam_rz;
            if (!m_so3_to_ypr(mat3 R, &cam_rz, &cam_ry, &cam_rx))
            {
                printf("[line_counter] Failed to convert rotation matrix to YPR\n");
                line_counter_using_jpg = false; // unlock
                continue;
            }
        }

        // DETECT GRID
        {
            asc_GridOptions options  = {0};
            options.correct_fisheye  = true;
            options.sobel_threshold  = sobel_threshold;
            options.maxima_threshold = maxima_threshold;
            options.max_error        = max_error;
            options.tile_width       = tile_width;
            options.fisheye_f        = fisheye_f;
            options.fisheye_center_x = fisheye_center_x;
            options.fisheye_center_y = fisheye_center_y;
            options.pinhole_fov_x    = pinhole_fov_x;
            options.pinhole_center_x = options.fisheye_center_x;
            options.pinhole_center_y = options.fisheye_center_y;
            options.height_adaptation_rate = 1.0f;

            asc_GridResult result =
            asc_find_grid(gray,
                          width,
                          height,
                          camera_roll,
                          camera_pitch,
                          camera_z,
                          options);

            if (result.error < MAX_ERROR)
            {
                // Note: The grid detector finds the yaw of the camera relative the grid.
                // Assuming no pitch and roll
                //   cam_rz (yaw of camera) = imu_rz (yaw of drone) + cam_imu_rz (yaw of cam/drone)
                // Hence
                //   imu_rz = cam_rz - cam_imu_rz

                // Todo: If there is pitch and roll, this calculation is actually wrong!
                // Todo: Is related to position fuser and mavros/vision_pose input
                // Todo: Cannot simply convert line counter yaw output into a quaternion
                // with zero pitch and roll, and pass that in.

                ascend_msgs::LineCounter msg;
                msg.timestamp = getnsec();
                msg.x1 = result.x[0];
                msg.y1 = result.y[0];
                msg.yaw1 = asci_angle(result.yaw[0] - cam_imu_rz);
                msg.x2 = result.x[1];
                msg.y2 = result.y[1];
                msg.yaw2 = asci_angle(result.yaw[1] - cam_imu_rz);
                msg.x3 = result.x[2];
                msg.y3 = result.y[2];
                msg.yaw3 = asci_angle(result.yaw[2] - cam_imu_rz);
                msg.x4 = result.x[3];
                msg.y4 = result.y[3];
                msg.yaw4 = asci_angle(result.yaw[3] - cam_imu_rz);
                pub.publish(msg);
            }
            else
            {
                ascend_msgs::LineCounter msg;
                msg.timestamp = getnsec();
                msg.x1 = 0.5f;
                msg.y1 = 0.5f;
                msg.yaw1 = 0.0f;
                msg.x2 = 0.5f;
                msg.y2 = 0.5f;
                msg.yaw2 = asci_angle(ASCI_PI/2.0f);
                msg.x3 = 0.5f;
                msg.y3 = 0.5f;
                msg.yaw3 = asci_angle(ASCI_PI);
                msg.x4 = 0.5f,
                msg.y4 = 0.5f;
                msg.yaw4 = asci_angle(3.0f*ASCI_PI/2.0f);
                pub.publish(msg);
            }
        }

        line_counter_using_jpg = false;
    }
}
#endif
#endif
