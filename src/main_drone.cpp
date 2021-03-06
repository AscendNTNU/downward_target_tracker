#include "parameters.h"

 
#if DUMMY_IMAGE==1
// This includes the binary data of the JPEG directly
// through the variables mock_jpg and mock_jpg_len. Nice
// for avoiding the relative/absolute file path madness.
// Create your own by running "xxd -i" on an image.
#include "mock_jpg.h"
#endif

#if TESTING_WITH_LAPTOP==1
// My laptop does not have a fisheye camera
// so I define different projection functions
// if I am testing with my laptop or not.
// These functions are defined in so_math.h
#define camera_project m_project_pinhole
#define camera_inverse_project m_ray_pinhole
#else
#define camera_project m_project_equidistant
#define camera_inverse_project m_ray_equidistant
#endif

#include <signal.h>
#include <assert.h>
#include <stdint.h>
#include <time.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

#include <downward_target_tracker/image.h>
#include <downward_target_tracker/info.h>
#include <downward_target_tracker/tracks.h>

#include "asc_usbcam.h"
#include "asc_tracker.h"
#include "mjpg_to_jpg.h"

// These parameters are sent out as msg/info.msg
// under the topic INFO_TOPIC (see parameters.h).
// They can be modified live by sending std_msgs::Float32
// on /target_debug/camera_f, ... etc
float _camera_f          = CAMERA_F_INIT;
float _camera_u0         = CAMERA_U0_INIT;
float _camera_v0         = CAMERA_V0_INIT;
float _cam_imu_rx        = CAM_IMU_RX_INIT;
float _cam_imu_ry        = CAM_IMU_RY_INIT;
float _cam_imu_rz        = CAM_IMU_RZ_INIT;
float _cam_imu_tx        = CAM_IMU_TX_INIT;
float _cam_imu_ty        = CAM_IMU_TY_INIT;
float _cam_imu_tz        = CAM_IMU_TZ_INIT;
float _r_g               = R_G_INIT;
float _r_b               = R_B_INIT;
float _r_n               = R_N_INIT;
float _g_r               = G_R_INIT;
float _g_b               = G_B_INIT;
float _g_n               = G_N_INIT;
float _white_threshold_r = WHITE_THRESHOLD_R_INIT;
float _white_threshold_g = WHITE_THRESHOLD_G_INIT;
float _white_threshold_b = WHITE_THRESHOLD_B_INIT;
float _white_threshold_d = WHITE_THRESHOLD_D_INIT;
float _pinhole_fov_x     = PINHOLE_FOV_X_INIT;
float _sobel_threshold   = SOBEL_THRESHOLD_INIT;
float _maxima_threshold  = MAXIMA_THRESHOLD_INIT;
float _max_error         = MAX_ERROR_INIT;
float _tile_width        = TILE_WIDTH_INIT;

#if USE_CAMERA_NODE == 1
//Stores information about the image data from the callback
sensor_msgs::CompressedImagePtr _image = boost::make_shared<sensor_msgs::CompressedImage>();
volatile float _image_available = false; // variable to keep track of if callback has made the image available
pthread_mutex_t image_mutex;
pthread_cond_t  image_condition;
#endif

// These describe the latest pose (roll, pitch, yaw, x, y, z)
// of the drone relative to the grid, and are updated in
// callback_imu. The target tracker uses *all* of these, while
// the line counter does not care about imu_rz, imu_tx or imu_ty.
float _imu_rx = 0.0f;
float _imu_ry = 0.0f;
float _imu_rz = 0.0f;
float _imu_tx = 0.0f;
float _imu_ty = 0.0f;
float _imu_tz = 1.0f;

void callback_camera_f(std_msgs::Float32 msg)          { _camera_f = msg.data; }
void callback_camera_u0(std_msgs::Float32 msg)         { _camera_u0 = msg.data; }
void callback_camera_v0(std_msgs::Float32 msg)         { _camera_v0 = msg.data; }
void callback_cam_imu_rx(std_msgs::Float32 msg)        { _cam_imu_rx = msg.data; }
void callback_cam_imu_ry(std_msgs::Float32 msg)        { _cam_imu_ry = msg.data; }
void callback_cam_imu_rz(std_msgs::Float32 msg)        { _cam_imu_rz = msg.data; }
void callback_cam_imu_tx(std_msgs::Float32 msg)        { _cam_imu_tx = msg.data; }
void callback_cam_imu_ty(std_msgs::Float32 msg)        { _cam_imu_ty = msg.data; }
void callback_cam_imu_tz(std_msgs::Float32 msg)        { _cam_imu_tz = msg.data; }
void callback_r_g(std_msgs::Float32 msg)               { _r_g = msg.data; }
void callback_r_b(std_msgs::Float32 msg)               { _r_b = msg.data; }
void callback_r_n(std_msgs::Float32 msg)               { _r_n = msg.data; }
void callback_g_r(std_msgs::Float32 msg)               { _g_r = msg.data; }
void callback_g_b(std_msgs::Float32 msg)               { _g_b = msg.data; }
void callback_g_n(std_msgs::Float32 msg)               { _g_n = msg.data; }
void callback_white_threshold_r(std_msgs::Float32 msg) { _white_threshold_r = msg.data; }
void callback_white_threshold_g(std_msgs::Float32 msg) { _white_threshold_g = msg.data; }
void callback_white_threshold_b(std_msgs::Float32 msg) { _white_threshold_b = msg.data; }
void callback_white_threshold_d(std_msgs::Float32 msg) { _white_threshold_d = msg.data; }
void callback_pinhole_fov_x(std_msgs::Float32 msg)     { _pinhole_fov_x = msg.data; }
void callback_sobel_threshold(std_msgs::Float32 msg)   { _sobel_threshold = msg.data; }
void callback_maxima_threshold(std_msgs::Float32 msg)  { _maxima_threshold = msg.data; }
void callback_max_error(std_msgs::Float32 msg)         { _max_error = msg.data; }
void callback_tile_width(std_msgs::Float32 msg)        { _tile_width = msg.data; }


void callback_imu(geometry_msgs::PoseStamped msg)
{
    m_quat_to_ypr(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w, &_imu_rz, &_imu_ry, &_imu_rx);
    _imu_tx = msg.pose.position.x;
    _imu_ty = msg.pose.position.y;
    _imu_tz = msg.pose.position.z;
}

#if USE_CAMERA_NODE == 1
// callback to be added to user queue instead of internal ROS queue
void callback_camera_img(const sensor_msgs::CompressedImageConstPtr msg) 
{
    pthread_mutex_lock(&image_mutex);

    //copy all info from msg to _image
    _image->header   = msg->header;
    _image->format   = msg->format;
    _image->data     = msg->data;
    _image_available = true;

    pthread_cond_signal(&image_condition);
    pthread_mutex_unlock(&image_mutex);
}
#endif

uint64_t getnsec()
{
    struct timespec ts = {};
    clock_gettime(CLOCK_REALTIME, &ts);
    uint64_t result = ((uint64_t)ts.tv_sec)*1000000000 +
                      ((uint64_t)ts.tv_nsec);
    return result;
}

// Oh no! The line counter also needs the fisheye
// camera. For efficiency we therefore run the line
// counter in the same program so that we can share
// memory quickly. It runs in a seperate thread, and
// the main thread below memcpy'ies the latest camera
// frame into a buffer it can access. There is a dubious
// mutex locking scheme to ensure no memcpy occurs while
// the grid detector is processing the frame.
//   The line counter uses the same camera/imu calibration
// as the target tracker (unsurprisingly), as well as the
// latest imu pose.
#include "main_line_counter.cpp"

void ctrlc(int)
{
    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "downward_target_tracker");
    ros::NodeHandle node;

    ros::Publisher pub_image  = node.advertise<downward_target_tracker::image>(IMAGE_TOPIC, 1);
    ros::Publisher pub_info   = node.advertise<downward_target_tracker::info>(INFO_TOPIC, 1);
    #if USE_TRACKER == 1
    ros::Publisher pub_tracks = node.advertise<downward_target_tracker::tracks>(TRACKS_TOPIC, 1);
    #endif

    ros::Subscriber sub_camera_f          = node.subscribe("/target_debug/camera_f",           1, callback_camera_f);
    ros::Subscriber sub_camera_u0         = node.subscribe("/target_debug/camera_u0",          1, callback_camera_u0);
    ros::Subscriber sub_camera_v0         = node.subscribe("/target_debug/camera_v0",          1, callback_camera_v0);
    ros::Subscriber sub_cam_imu_rx        = node.subscribe("/target_debug/cam_imu_rx",         1, callback_cam_imu_rx);
    ros::Subscriber sub_cam_imu_ry        = node.subscribe("/target_debug/cam_imu_ry",         1, callback_cam_imu_ry);
    ros::Subscriber sub_cam_imu_rz        = node.subscribe("/target_debug/cam_imu_rz",         1, callback_cam_imu_rz);
    ros::Subscriber sub_cam_imu_tx        = node.subscribe("/target_debug/cam_imu_tx",         1, callback_cam_imu_tx);
    ros::Subscriber sub_cam_imu_ty        = node.subscribe("/target_debug/cam_imu_ty",         1, callback_cam_imu_ty);
    ros::Subscriber sub_cam_imu_tz        = node.subscribe("/target_debug/cam_imu_tz",         1, callback_cam_imu_tz);
    ros::Subscriber sub_r_g               = node.subscribe("/target_debug/r_g",                1, callback_r_g);
    ros::Subscriber sub_r_b               = node.subscribe("/target_debug/r_b",                1, callback_r_b);
    ros::Subscriber sub_r_n               = node.subscribe("/target_debug/r_n",                1, callback_r_n);
    ros::Subscriber sub_g_r               = node.subscribe("/target_debug/g_r",                1, callback_g_r);
    ros::Subscriber sub_g_b               = node.subscribe("/target_debug/g_b",                1, callback_g_b);
    ros::Subscriber sub_g_n               = node.subscribe("/target_debug/g_n",                1, callback_g_n);
    ros::Subscriber sub_white_threshold_r = node.subscribe("/target_debug/white_threshold_r",  1, callback_white_threshold_r);
    ros::Subscriber sub_white_threshold_g = node.subscribe("/target_debug/white_threshold_g",  1, callback_white_threshold_g);
    ros::Subscriber sub_white_threshold_b = node.subscribe("/target_debug/white_threshold_b",  1, callback_white_threshold_b);
    ros::Subscriber sub_white_threshold_d = node.subscribe("/target_debug/white_threshold_d",  1, callback_white_threshold_d);
    ros::Subscriber sub_pinhole_fov_x     = node.subscribe("/target_debug/pinhole_fov_x",      1, callback_pinhole_fov_x);
    ros::Subscriber sub_sobel_threshold   = node.subscribe("/target_debug/sobel_threshold",    1, callback_sobel_threshold);
    ros::Subscriber sub_maxima_threshold  = node.subscribe("/target_debug/maxima_threshold",   1, callback_maxima_threshold);
    ros::Subscriber sub_max_error         = node.subscribe("/target_debug/max_error",          1, callback_max_error);
    ros::Subscriber sub_tile_width        = node.subscribe("/target_debug/tile_width",         1, callback_tile_width);


    ros::Subscriber sub_imu = node.subscribe(IMU_POSE_TOPIC, 1, callback_imu);
    
    #if USE_CAMERA_NODE == 1
    pthread_mutex_init(&image_mutex, NULL);
    pthread_cond_init(&image_condition, NULL);

    //We create our own image queue that will be used in an asynchronous spinner
    ros::CallbackQueue img_queue; 
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<sensor_msgs::CompressedImage>(
        CAMERA_TOPIC, 
        1, 
        callback_camera_img, 
        ros::VoidPtr(), 
        &img_queue 
        );
    ros::Subscriber sub_img = node.subscribe(ops);

    // spawn async spinner with 1 thread, running on our image queue
    ros::AsyncSpinner async_spinner(1, &img_queue);
    async_spinner.start();
    #endif

    #if RUN_LINE_COUNTER==1
    line_counter_init(&node);
    #endif

    signal(SIGINT, ctrlc);

    #if DUMMY_IMAGE == 0 && USE_CAMERA_NODE == 0
    {
        usbcam_opt_t opt = {0};
        opt.device_name = DEVICE_NAME;
        opt.pixel_format = V4L2_PIX_FMT_MJPEG;
        opt.width = CAMERA_WIDTH;
        opt.height = CAMERA_HEIGHT;
        opt.buffers = CAMERA_BUFFERS;
        usbcam_init(opt);
    }
    #endif

    const int Ix = CAMERA_WIDTH>>CAMERA_LEVELS;
    const int Iy = CAMERA_HEIGHT>>CAMERA_LEVELS;
    static unsigned char I[CAMERA_WIDTH*CAMERA_HEIGHT*3];
    uint64_t t_begin = getnsec();
    for (int frame = 0;; frame++)
    {
        // RECEIVE LATEST IMAGE
        #if USE_CAMERA_NODE == 1
        pthread_mutex_lock(&image_mutex);
        while(!_image_available) 
        {
            pthread_cond_wait(&image_condition, 
                              &image_mutex);
        }
        _image_available = false;
        #endif

        unsigned char *jpg_data = 0;
        unsigned int jpg_size = 0;
        int img_width, img_height;
        timeval timestamp = {0};
        {
            #if DUMMY_IMAGE == 1
            jpg_data = mock_jpg;
            jpg_size = mock_jpg_len;
            usleep(5*1000);
            #elif TESTING_WITH_LAPTOP == 1
            usbcam_lock_mjpg(&jpg_data, &jpg_size, &timestamp);
            #elif USE_CAMERA_NODE == 0
            usbcam_lock(&jpg_data, &jpg_size, &timestamp);
            #elif USE_CAMERA_NODE == 1
            jpg_data   = _image->data.data();
            jpg_size   = _image->data.size(); 
            timestamp.tv_sec  = _image->header.stamp.sec;
            timestamp.tv_usec = _image->header.stamp.nsec * 1000;
            if(!jpg_size) continue; //invalid picture
            #endif
        }

        #if USE_CAMERA_NODE == 1
        pthread_mutex_unlock(&image_mutex);
        #endif

        // SHARE IMAGE WITH LINE COUNTER
        float dt_memcpy = 0.0f;
        #if RUN_LINE_COUNTER==1
        line_counter_copy_image(jpg_data, jpg_size, &dt_memcpy);
        #endif

        // MEASURE TIME BETWEEN WHEN IMAGES WERE TAKEN
        float dt_frame = 0.0f;
        {
            uint64_t sec = (uint64_t)timestamp.tv_sec;
            uint64_t usec = (uint64_t)timestamp.tv_usec;
            uint64_t t = sec*1000*1000 + usec;
            static uint64_t last_t = t;
            dt_frame = (t-last_t)/1e6;
            last_t = t;
        }

        // DECOMPRESS AND CONVERT JPEG TO RGB
        float dt_jpeg_to_rgb = 0.0f;
        {
            uint64_t t1 = getnsec();
            if (!usbcam_jpeg_to_rgb(Ix, Iy, I, jpg_data, jpg_size))
            {
                #if USE_CAMERA_NODE == 0
                usbcam_unlock();
                #endif
                continue;
            }
            uint64_t t2 = getnsec();
            dt_jpeg_to_rgb = (t2-t1)/1e9;
        }

        // GET LATEST MESSAGES BEFORE PROCESSING IMAGE
        #if RUN_LINE_COUNTER==1
        pthread_mutex_lock(&line_counter_param_mutex);
        #endif
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
        #if RUN_LINE_COUNTER==1
        pthread_mutex_unlock(&line_counter_param_mutex);
        #endif

        // COMPUTE CAMERA POSE RELATIVE GRID
        mat3 imu_rot = m_rotz(imu_rz)*m_roty(imu_ry)*m_rotx(imu_rx);
        vec3 imu_pos = m_vec3(imu_tx, imu_ty, imu_tz);
        mat3 cam_imu_rot = m_rotz(cam_imu_rz)*m_roty(cam_imu_ry)*m_rotx(cam_imu_rx);
        vec3 cam_imu_pos = m_vec3(cam_imu_tx, cam_imu_ty, cam_imu_tz);
        mat3 rot = imu_rot*cam_imu_rot;
        vec3 pos = imu_pos + imu_rot*cam_imu_pos;

        // CALCULATE CAMERA PARAMETERS FOR DOWNSCALED IMAGE
        float f = camera_f*Ix/CAMERA_WIDTH;
        float u0 = camera_u0*Ix/CAMERA_WIDTH;
        float v0 = camera_v0*Ix/CAMERA_WIDTH;

        // DETECT AND TRACK TARGETS
        tracks_t tracks = {0};
        float dt_track_targets = 0.0f;
        #if USE_TRACKER == 1
        {
            uint64_t t1 = getnsec();
            track_targets_opt_t opt = {0};
            opt.r_g = r_g;
            opt.r_b = r_b;
            opt.r_n = r_n;
            opt.g_r = g_r;
            opt.g_b = g_b;
            opt.g_n = g_n;
            opt.f = f;
            opt.u0 = u0;
            opt.v0 = v0;
            opt.I = I;
            opt.Ix = Ix;
            opt.Iy = Iy;
            opt.rot = rot;
            opt.pos = pos;
            opt.timestamp = (getnsec()-t_begin)/1e9;
            tracks = track_targets(opt);
            uint64_t t2 = getnsec();
            dt_track_targets = (t2-t1)/1e9;
        }
        #endif 

        // MEASURE TIME BETWEEN EACH OUTPUT
        float dt_cycle = 0.0f;
        {
            static uint64_t t1 = getnsec();
            uint64_t t2 = getnsec();
            dt_cycle = (t2-t1)/1e9;
            t1 = t2;
        }

        // PUBLISH STUFF
        {
            #if USE_TRACKER == 1
            // PUBLISH TARGET TRACKS
            {
                downward_target_tracker::tracks msg;
                msg.num_targets = tracks.num_targets;
                for (int i = 0; i < tracks.num_targets; i++)
                {
                    msg.unique_id.push_back(tracks.targets[i].unique_id);
                    msg.position_x.push_back(tracks.targets[i].last_seen.x);
                    msg.position_y.push_back(tracks.targets[i].last_seen.y);
                    msg.velocity_x.push_back(tracks.targets[i].velocity_x);
                    msg.velocity_y.push_back(tracks.targets[i].velocity_y);
                    msg.detection_rate.push_back(tracks.targets[i].detection_rate);
                }
                msg.observed_180 = tracks.observed_180;
                msg.time_until_180 = tracks.time_until_180;
                pub_tracks.publish(msg);
            }
            #endif

            static uint64_t t_last_info = getnsec();

            // PUBLISH INFO
            if ((getnsec() - t_last_info)/1e9 > INFO_PUBLISH_INTERVAL)
            {
                t_last_info = getnsec();

                downward_target_tracker::info msg;
                msg.dt_frame = dt_frame;
                msg.dt_jpeg_to_rgb = dt_jpeg_to_rgb;
                msg.dt_track_targets = dt_track_targets;
                msg.dt_cycle = dt_cycle;

                msg.line_counter_dt_jpeg_to_rgb = line_counter_dt_jpeg_to_rgb;
                msg.line_counter_dt_threshold = line_counter_dt_threshold;
                msg.line_counter_dt_find_grid = line_counter_dt_find_grid;

                msg.camera_f = camera_f;
                msg.camera_u0 = camera_u0;
                msg.camera_v0 = camera_v0;
                msg.cam_imu_rx = cam_imu_rx;
                msg.cam_imu_ry = cam_imu_ry;
                msg.cam_imu_rz = cam_imu_rz;
                msg.cam_imu_tx = cam_imu_tx;
                msg.cam_imu_ty = cam_imu_ty;
                msg.cam_imu_tz = cam_imu_tz;
                msg.r_g = r_g;
                msg.r_b = r_b;
                msg.r_n = r_n;
                msg.g_r = g_r;
                msg.g_b = g_b;
                msg.g_n = g_n;
                msg.white_threshold_r = white_threshold_r;
                msg.white_threshold_g = white_threshold_g;
                msg.white_threshold_b = white_threshold_b;
                msg.white_threshold_d = white_threshold_d;
                msg.pinhole_fov_x = pinhole_fov_x;
                msg.sobel_threshold = sobel_threshold;
                msg.maxima_threshold = maxima_threshold;
                msg.max_error = max_error;
                msg.tile_width = tile_width;

                msg.imu_rx = imu_rx;
                msg.imu_ry = imu_ry;
                msg.imu_rz = imu_rz;
                msg.imu_tx = imu_tx;
                msg.imu_ty = imu_ty;
                msg.imu_tz = imu_tz;

                // debug info
                msg.camera_w = CAMERA_WIDTH;
                msg.camera_h = CAMERA_HEIGHT;
                msg.image_x = Ix;
                msg.image_y = Iy;
                #if USE_TRACKER == 1
                // tracks
                msg.num_targets = tracks.num_targets;
                for (int i = 0; i < tracks.num_targets; i++)
                {
                    msg.unique_id.push_back(tracks.targets[i].unique_id);
                    msg.position_x.push_back(tracks.targets[i].last_seen.x);
                    msg.position_y.push_back(tracks.targets[i].last_seen.y);
                    msg.velocity_x.push_back(tracks.targets[i].velocity_x);
                    msg.velocity_y.push_back(tracks.targets[i].velocity_y);
                    msg.detection_rate.push_back(tracks.targets[i].detection_rate);
                }
                // last_seen
                for (int i = 0; i < tracks.num_targets; i++)
                {
                    msg.last_seen_u.push_back(tracks.targets[i].last_seen.u);
                    msg.last_seen_v.push_back(tracks.targets[i].last_seen.v);
                    msg.last_seen_u1.push_back(tracks.targets[i].last_seen.u1);
                    msg.last_seen_v1.push_back(tracks.targets[i].last_seen.v1);
                    msg.last_seen_u2.push_back(tracks.targets[i].last_seen.u2);
                    msg.last_seen_v2.push_back(tracks.targets[i].last_seen.v2);
                    msg.last_seen_x.push_back(tracks.targets[i].last_seen.x);
                    msg.last_seen_y.push_back(tracks.targets[i].last_seen.y);
                    msg.last_seen_t.push_back(tracks.targets[i].last_seen.t);
                }
                // detections
                msg.num_detections = tracks.num_detections;
                for (int i = 0; i < tracks.num_detections; i++)
                {
                    msg.detection_u.push_back(tracks.detections[i].u);
                    msg.detection_v.push_back(tracks.detections[i].v);
                    msg.detection_u1.push_back(tracks.detections[i].u1);
                    msg.detection_v1.push_back(tracks.detections[i].v1);
                    msg.detection_u2.push_back(tracks.detections[i].u2);
                    msg.detection_v2.push_back(tracks.detections[i].v2);
                }
                #endif
                pub_info.publish(msg);
            }

            static uint64_t t_last_image = getnsec();

            // PUBLISH JPEG
            if ((getnsec() - t_last_image)/1e9 > IMAGE_PUBLISH_INTERVAL)
            {
                t_last_image = getnsec();

                downward_target_tracker::image msg;
                msg.jpg_data.resize(jpg_size);
                msg.width = CAMERA_WIDTH;
                msg.height = CAMERA_HEIGHT;
                memcpy(&msg.jpg_data[0], jpg_data, jpg_size);
                pub_image.publish(msg);
            }
        }

        #if DUMMY_IMAGE == 0 && USE_CAMERA_NODE == 0
        usbcam_unlock();
        #endif
    }

    return 0;
}
