#define DUMMY_IMAGE            0
#define DEVICE_NAME            "/dev/video0"
#define IMU_POSE_TOPIC         "/mavros/vision_pose/pose"
#define INFO_PUBLISH_INTERVAL  0.0f // minimum required seconds between publications (set to zero to publish every frame)
#define IMAGE_PUBLISH_INTERVAL 0.0f // minimum required seconds between publications (set to zero to publish every frame)

// Camera rotation offset (R_cam^imu = Rz(cam_imu_rz)Ry(cam_imu_ry)Rx(cam_imu_rx)) [camera to imu frame]
#define CAM_IMU_RX_INIT    0.0f
#define CAM_IMU_RY_INIT    0.0f
#define CAM_IMU_RZ_INIT    0.0f

// Camera position offset (T_cam/imu^imu = {cam_imu_tx, cam_imu_ty, cam_imu_tz}) [camera relative imu in imu frame]
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

//
// Implementation
//

#include "camera_define.h"

#if DUMMY_IMAGE==0 && TESTING_WITH_LAPTOP==0
#define USBCAM_DEBUG       1
#define CAMERA_WIDTH       800
#define CAMERA_HEIGHT      600
#define CAMERA_BUFFERS     3
#define CAMERA_LEVELS      2 // Downscale factor (0=none, 1=half, 2=quarter)
#define CAMERA_F_INIT      434.0f
#define CAMERA_U0_INIT     375.0f
#define CAMERA_V0_INIT     275.0f
#endif

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

#if DUMMY_IMAGE==1
#include "mock_jpg.h"      // hint: run "xxd -i" on an image to generate a header-embedded binary
#define CAMERA_WIDTH       1280
#define CAMERA_HEIGHT      720
#define CAMERA_LEVELS      3
#define CAMERA_F_INIT      494.0f
#define CAMERA_U0_INIT     649.0f
#define CAMERA_V0_INIT     335.0f
#endif

#include <signal.h>
#include <assert.h>
#include <stdint.h>
#include <time.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <downward_target_tracker/image.h>
#include <downward_target_tracker/info.h>
#include <downward_target_tracker/tracks.h>
#include "asc_usbcam.h"
#include "asc_tracker.h"
#include "mjpg_to_jpg.h"

float camera_f = CAMERA_F_INIT;
float camera_u0 = CAMERA_U0_INIT;
float camera_v0 = CAMERA_V0_INIT;
float cam_imu_rx = CAM_IMU_RX_INIT;
float cam_imu_ry = CAM_IMU_RY_INIT;
float cam_imu_rz = CAM_IMU_RZ_INIT;
float cam_imu_tx = CAM_IMU_TX_INIT;
float cam_imu_ty = CAM_IMU_TY_INIT;
float cam_imu_tz = CAM_IMU_TZ_INIT;
float r_g = R_G_INIT;
float r_b = R_B_INIT;
float r_n = R_N_INIT;
float g_r = G_R_INIT;
float g_b = G_B_INIT;
float g_n = G_N_INIT;

float imu_rx = 0.0f;
float imu_ry = 0.0f;
float imu_rz = 0.0f;
float imu_tx = 0.0f;
float imu_ty = 0.0f;
float imu_tz = 1.0f;

void callback_camera_f(std_msgs::Float32 msg) { camera_f = msg.data; }
void callback_camera_u0(std_msgs::Float32 msg) { camera_u0 = msg.data; }
void callback_camera_v0(std_msgs::Float32 msg) { camera_v0 = msg.data; }
void callback_cam_imu_rx(std_msgs::Float32 msg) { cam_imu_rx = msg.data; }
void callback_cam_imu_ry(std_msgs::Float32 msg) { cam_imu_ry = msg.data; }
void callback_cam_imu_rz(std_msgs::Float32 msg) { cam_imu_rz = msg.data; }
void callback_cam_imu_tx(std_msgs::Float32 msg) { cam_imu_tx = msg.data; }
void callback_cam_imu_ty(std_msgs::Float32 msg) { cam_imu_ty = msg.data; }
void callback_cam_imu_tz(std_msgs::Float32 msg) { cam_imu_tz = msg.data; }
void callback_r_g(std_msgs::Float32 msg) { r_g = msg.data; }
void callback_r_b(std_msgs::Float32 msg) { r_b = msg.data; }
void callback_r_n(std_msgs::Float32 msg) { r_n = msg.data; }
void callback_g_r(std_msgs::Float32 msg) { g_r = msg.data; }
void callback_g_b(std_msgs::Float32 msg) { g_b = msg.data; }
void callback_g_n(std_msgs::Float32 msg) { g_n = msg.data; }
void callback_imu(geometry_msgs::PoseStamped msg)
{
    m_quat_to_ypr(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w, &imu_rz, &imu_ry, &imu_rx);
    imu_tx = msg.pose.position.x;
    imu_ty = msg.pose.position.y;
    imu_tz = msg.pose.position.z;
}

uint64_t getnsec()
{
    struct timespec ts = {};
    clock_gettime(CLOCK_REALTIME, &ts);
    uint64_t result = ((uint64_t)ts.tv_sec)*1000000000 +
                      ((uint64_t)ts.tv_nsec);
    return result;
}

void ctrlc(int)
{
    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "downward_target_tracker");
    ros::NodeHandle node;
    ros::Publisher pub_image = node.advertise<downward_target_tracker::image>("downward_target_tracker/image", 1);
    ros::Publisher pub_info = node.advertise<downward_target_tracker::info>("downward_target_tracker/info", 1);
    ros::Publisher pub_tracks = node.advertise<downward_target_tracker::tracks>("downward_target_tracker/tracks", 1);
    ros::Subscriber sub_camera_f = node.subscribe("/downward_target_debug/camera_f", 1, callback_camera_f);
    ros::Subscriber sub_camera_u0 = node.subscribe("/downward_target_debug/camera_u0", 1, callback_camera_u0);
    ros::Subscriber sub_camera_v0 = node.subscribe("/downward_target_debug/camera_v0", 1, callback_camera_v0);
    ros::Subscriber sub_cam_imu_rx = node.subscribe("/downward_target_debug/cam_imu_rx", 1, callback_cam_imu_rx);
    ros::Subscriber sub_cam_imu_ry = node.subscribe("/downward_target_debug/cam_imu_ry", 1, callback_cam_imu_ry);
    ros::Subscriber sub_cam_imu_rz = node.subscribe("/downward_target_debug/cam_imu_rz", 1, callback_cam_imu_rz);
    ros::Subscriber sub_cam_imu_tx = node.subscribe("/downward_target_debug/cam_imu_tx", 1, callback_cam_imu_tx);
    ros::Subscriber sub_cam_imu_ty = node.subscribe("/downward_target_debug/cam_imu_ty", 1, callback_cam_imu_ty);
    ros::Subscriber sub_cam_imu_tz = node.subscribe("/downward_target_debug/cam_imu_tz", 1, callback_cam_imu_tz);
    ros::Subscriber sub_r_g = node.subscribe("/downward_target_debug/r_g", 1, callback_r_g);
    ros::Subscriber sub_r_b = node.subscribe("/downward_target_debug/r_b", 1, callback_r_b);
    ros::Subscriber sub_r_n = node.subscribe("/downward_target_debug/r_n", 1, callback_r_n);
    ros::Subscriber sub_g_r = node.subscribe("/downward_target_debug/g_r", 1, callback_g_r);
    ros::Subscriber sub_g_b = node.subscribe("/downward_target_debug/g_b", 1, callback_g_b);
    ros::Subscriber sub_g_n = node.subscribe("/downward_target_debug/g_n", 1, callback_g_n);
    ros::Subscriber sub_imu = node.subscribe(IMU_POSE_TOPIC, 1, callback_imu);

    signal(SIGINT, ctrlc);

    #if DUMMY_IMAGE==0
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
        unsigned char *jpg_data = 0;
        unsigned int jpg_size = 0;
        timeval timestamp = {0};
        {
            #if DUMMY_IMAGE==1
            jpg_data = mock_jpg;
            jpg_size = mock_jpg_len;
            usleep(16*1000);
            #elif TESTING_WITH_LAPTOP==1
            usbcam_lock_mjpg(&jpg_data, &jpg_size, &timestamp);
            #else
            usbcam_lock(&jpg_data, &jpg_size, &timestamp);
            #endif
        }

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

        // DECOMPRESS
        float dt_jpeg_to_rgb = 0.0f;
        {
            uint64_t t1 = getnsec();
            if (!usbcam_jpeg_to_rgb(Ix, Iy, I, jpg_data, jpg_size))
            {
                usbcam_unlock();
                continue;
            }
            uint64_t t2 = getnsec();
            dt_jpeg_to_rgb = (t2-t1)/1e9;
        }

        // GET LATEST MESSAGES BEFORE PROCESSING IMAGE
        ros::spinOnce();

        // RUN ONE ITERATION OF TARGET DETECTION AND TRACKING
        mat3 imu_rot = m_rotz(imu_rz)*m_roty(imu_ry)*m_rotx(imu_rx);
        vec3 imu_pos = m_vec3(imu_tx, imu_ty, imu_tz);
        mat3 cam_imu_rot = m_rotz(cam_imu_rz)*m_roty(cam_imu_ry)*m_rotx(cam_imu_rx);
        vec3 cam_imu_pos = m_vec3(cam_imu_tx, cam_imu_ty, cam_imu_tz);
        mat3 rot = imu_rot*cam_imu_rot;
        vec3 pos = imu_pos + imu_rot*cam_imu_pos;
        float f = camera_f*Ix/CAMERA_WIDTH;
        float u0 = camera_u0*Ix/CAMERA_WIDTH;
        float v0 = camera_v0*Ix/CAMERA_WIDTH;
        tracks_t tracks = {0};
        float dt_track_targets = 0.0f;
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

                msg.camera_f = camera_f;
                msg.camera_u0 = camera_u0;
                msg.camera_v0 = camera_v0;
                msg.camera_w = CAMERA_WIDTH;
                msg.camera_h = CAMERA_HEIGHT;

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

                msg.imu_rx = imu_rx;
                msg.imu_ry = imu_ry;
                msg.imu_rz = imu_rz;
                msg.imu_tx = imu_tx;
                msg.imu_ty = imu_ty;
                msg.imu_tz = imu_tz;

                // debug info
                msg.image_x = Ix;
                msg.image_y = Iy;
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

                pub_info.publish(msg);
            }

            static uint64_t t_last_image = getnsec();

            // PUBLISH JPEG
            if ((getnsec() - t_last_image)/1e9 > IMAGE_PUBLISH_INTERVAL)
            {
                t_last_image = getnsec();

                downward_target_tracker::image msg;
                msg.jpg_data.resize(jpg_size);
                memcpy(&msg.jpg_data[0], jpg_data, jpg_size);
                pub_image.publish(msg);
            }
        }

        #if DUMMY_IMAGE==0
        usbcam_unlock();
        #endif
    }

    return 0;
}
