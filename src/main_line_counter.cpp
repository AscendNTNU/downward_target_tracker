#ifdef ASC_GRID_DEBUG
#include "vdb/vdb.h"
#endif
#define ASC_GRID_DETECTOR_IMPLEMENTATION

#if USE_SSE == 1
#define ASC_GRID_DETECTOR_SSE
#endif

#include "asc_grid_detector.h"
#include <pthread.h> 
#include <ascend_msgs/LineCounter.h>

// These need to be volatile, otherwise the main_line_counter thread did
// not work properly when I compile with optimizations (-O2).
volatile bool         line_counter_image_avail = false;
volatile unsigned int line_counter_jpg_size;
static unsigned char  line_counter_jpg_data[CAMERA_WIDTH*CAMERA_HEIGHT*3];
pthread_mutex_t       line_counter_image_mutex;
pthread_mutex_t       line_counter_param_mutex;
pthread_cond_t        line_counter_condition;
float                 line_counter_dt_jpeg_to_rgb = 0.0f;
float                 line_counter_dt_find_grid = 0.0f;
float                 line_counter_dt_threshold = 0.0f;

ros::Publisher pub_line_counter;

void *line_counter_main(void *);
void line_counter_init(ros::NodeHandle *node)
{
    pthread_mutex_init(&line_counter_image_mutex, NULL);
    pthread_mutex_init(&line_counter_param_mutex, NULL);
    pthread_cond_init(&line_counter_condition, NULL);

    pthread_t t;
    pthread_create(&t, NULL, line_counter_main, NULL);

    pub_line_counter = node->advertise<ascend_msgs::LineCounter>(LINE_COUNTER_TOPIC, 1);
}

void line_counter_copy_image(unsigned char *jpg_data, unsigned int jpg_size, float *out_dt_memcpy)
{
    float dt_memcpy = 0.0f;
    uint64_t t1 = getnsec();
    if (pthread_mutex_trylock(&line_counter_image_mutex) == 0)
    {

        memcpy(line_counter_jpg_data, jpg_data, jpg_size);
        line_counter_jpg_size = jpg_size;
        line_counter_image_avail = true;

        // Wakeup the line_counter thread that is waiting on the condition
        pthread_cond_signal(&line_counter_condition);
        pthread_mutex_unlock(&line_counter_image_mutex);
    }
    else
    {
        printf("[Warning] Line counter ran too slow and missed a camera frame. Send me a message if this occurs often.\n");
    }
    uint64_t t2 = getnsec();
    dt_memcpy = (t2-t1)/1e9;
    *out_dt_memcpy = dt_memcpy;
}

#if 1
void *line_counter_main(void *)
{
    printf("[line_counter] Running\n");
    const int Ix = CAMERA_WIDTH>>CAMERA_LEVELS_LINE_COUNTER;
    const int Iy = CAMERA_HEIGHT>>CAMERA_LEVELS_LINE_COUNTER;
    static unsigned char I_rgb[Ix*Iy*3];
    static unsigned char I_gray[Ix*Iy];
    for (int frame = 0;; frame++)
    {
        // WAIT UNTIL MAIN THREAD RECEIVES FRAME FROM CAMERA
        pthread_mutex_lock(&line_counter_image_mutex);
        while (!line_counter_image_avail)
        {
            pthread_cond_wait(&line_counter_condition, 
                              &line_counter_image_mutex);
        }
        line_counter_image_avail = false; // we have now 'consumed' this frame. main thread will set this to true.

        // DECOMPRESS AND CONVERT JPEG TO RGB
        float dt_jpeg_to_rgb = 0.0f;
        {
            uint64_t t1 = getnsec();
            if (!usbcam_jpeg_to_rgb(Ix, Iy, I_rgb, line_counter_jpg_data, line_counter_jpg_size))
            {
                printf("[line_counter] Failed to decompress JPEG\n");

                // RELEASE LOCK ON FRAME (allow main to update jpg)
                pthread_mutex_unlock(&line_counter_image_mutex);
                continue;
            }
            uint64_t t2 = getnsec();
            dt_jpeg_to_rgb = (t2-t1)/1e9;
        }

        // RELEASE LOCK ON FRAME (allow main to update jpg)
        pthread_mutex_unlock(&line_counter_image_mutex);

        // GET LATEST MESSAGES BEFORE PROCESSING IMAGE
        pthread_mutex_lock(&line_counter_param_mutex); // disallow main from modifying parameters
        ros::spinOnce();
        float camera_f = _camera_f;
        float camera_u0 = _camera_u0;
        float camera_v0 = _camera_v0;
        float cam_imu_rx = _cam_imu_rx;
        float cam_imu_ry = _cam_imu_ry;
        float cam_imu_rz = _cam_imu_rz;
        float cam_imu_tx = _cam_imu_tx;
        float cam_imu_ty = _cam_imu_ty;
        float cam_imu_tz = _cam_imu_tz;
        float r_g = _r_g;
        float r_b = _r_b;
        float r_n = _r_n;
        float g_r = _g_r;
        float g_b = _g_b;
        float g_n = _g_n;
        float white_threshold_r = _white_threshold_r;
        float white_threshold_g = _white_threshold_g;
        float white_threshold_b = _white_threshold_b;
        float white_threshold_d = _white_threshold_d;
        float pinhole_fov_x = _pinhole_fov_x;
        float sobel_threshold = _sobel_threshold;
        float maxima_threshold = _maxima_threshold;
        float max_error = _max_error;
        float tile_width = _tile_width;
        float imu_rx = _imu_rx;
        float imu_ry = _imu_ry;
        float imu_rz = _imu_rz;
        float imu_tx = _imu_tx;
        float imu_ty = _imu_ty;
        float imu_tz = _imu_tz;
        pthread_mutex_unlock(&line_counter_param_mutex); // allow main to modify parameters

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
        // AND EXTRACT PITCH, ROLL AND HEIGHT
        float cam_ry, cam_rx, cam_rz, cam_z;
        {
            mat3 imu_rot = m_rotz(imu_rz)*m_roty(imu_ry)*m_rotx(imu_rx);
            vec3 imu_pos = m_vec3(imu_tx, imu_ty, imu_tz);
            mat3 cam_imu_rot = m_rotz(cam_imu_rz)*m_roty(cam_imu_ry)*m_rotx(cam_imu_rx);
            vec3 cam_imu_pos = m_vec3(cam_imu_tx, cam_imu_ty, cam_imu_tz);
            mat3 rot = imu_rot*cam_imu_rot;
            vec3 pos = imu_pos + imu_rot*cam_imu_pos;

            cam_z = pos.z;
            if (!m_so3_to_ypr(rot, &cam_rz, &cam_ry, &cam_rx))
            {
                printf("[line_counter] Failed to convert rotation matrix to YPR.");
                continue;
            }
        }

        // DETECT GRID
        float dt_find_grid = 0.0f;
        {
            uint64_t t1 = getnsec();

            asc_GridOptions options  = {0};
            options.correct_fisheye  = true;
            options.sobel_threshold  = sobel_threshold;
            options.maxima_threshold = maxima_threshold;
            options.max_error        = max_error;
            options.tile_width       = tile_width;
            options.fisheye_f        = camera_f*Ix/CAMERA_WIDTH;
            options.fisheye_center_x = camera_u0*Ix/CAMERA_WIDTH;
            options.fisheye_center_y = camera_v0*Ix/CAMERA_WIDTH;
            options.pinhole_fov_x    = pinhole_fov_x;
            options.pinhole_center_x = options.fisheye_center_x;
            options.pinhole_center_y = options.fisheye_center_y;
            options.height_adaptation_rate = 1.0f;

            asc_GridResult result =
            asc_find_grid(I_gray,Ix,Iy, cam_rx,cam_ry,cam_z, options, I_rgb);

            ascend_msgs::LineCounter msg;
            if (result.error < max_error)
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
            }
            else
            {
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
            }
            pub_line_counter.publish(msg);

            #if RUN_LINE_COUNTER_FILTER==1
            filter_and_publish_vision_pose(msg);
            #endif

            uint64_t t2 = getnsec();
            dt_find_grid = (t2-t1)/1e9;
        }

        // SHARE TIMING STATISTICS WITH MAIN THREAD
        line_counter_dt_jpeg_to_rgb = dt_jpeg_to_rgb;
        line_counter_dt_find_grid = dt_find_grid;
        line_counter_dt_threshold = dt_threshold;
    }
}
#else
void *line_counter_main(void *)
{
    printf("line counter!\n");
    for (int frame = 0;; frame++)
    {
        while (!line_counter_image_avail)
        {
        }
        pthread_mutex_lock(&line_counter_image_mutex);
        line_counter_image_avail = false;

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

        // COPY PARAMETERS
        // (they might be modified by the main thread when it calls ros::spinOnce())
        pthread_mutex_lock(&line_counter_param_mutex);
        float _imu_rz = imu_rz;
        float _imu_ry = imu_ry;
        float _imu_rx = imu_rx;
        float _imu_tx = imu_tx;
        float _imu_ty = imu_ty;
        float _imu_tz = imu_tz;
        float _cam_imu_rz = cam_imu_rz;
        float _cam_imu_ry = cam_imu_ry;
        float _cam_imu_rx = cam_imu_rx;
        float _cam_imu_tx = cam_imu_tx;
        float _cam_imu_ty = cam_imu_ty;
        float _cam_imu_tz = cam_imu_tz;
        float _sobel_threshold = sobel_threshold;
        float _maxima_threshold = maxima_threshold;
        float _max_error = max_error;
        float _tile_width = tile_width;
        float _camera_f = camera_f;
        float _camera_u0 = camera_u0;
        float _camera_v0 = camera_v0;
        float _pinhole_fov_x = pinhole_fov_x;
        pthread_mutex_unlock(&line_counter_param_mutex);

        // publish test
        {
            ascend_msgs::LineCounter msg;
            msg.timestamp = getnsec();
            msg.x1 = 1; msg.y1 = 10; msg.yaw1 = 100;
            msg.x2 = 2; msg.y2 = 20; msg.yaw2 = 200;
            msg.x3 = 3; msg.y3 = 30; msg.yaw3 = 300;
            msg.x4 = 4; msg.y4 = 40; msg.yaw4 = 400;
            pub_line_counter.publish(msg);
        }

        printf("%d. %.2f ms\n", frame, 1000.0f*dt_fwrite);

        pthread_mutex_unlock(&image_mutex);
    }
}
#endif
