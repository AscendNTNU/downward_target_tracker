#define USBCAM_DEBUG
#define CAMERA_NAME        "/dev/video1"
#define CAMERA_WIDTH       800
#define CAMERA_HEIGHT      600
#define CAMERA_BUFFERS     3
#define CAMERA_LEVELS      2 // Downscale factor (0=none, 1=half, 2=quarter)

#include <time.h>
#include <signal.h>
#include <assert.h>
#include <stdint.h>
#include <ros/ros.h>
#include <downward_target_tracker/image.h>
#include <downward_target_tracker/info.h>
#include <downward_target_tracker/tracks.h>
#include <downward_target_debug/debug.h>

#include "asc_usbcam.h"
#include "asc_tracker.h"

uint64_t get_nanoseconds()
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

//
// Callbacks
//

float camera_f = 434.0f;
float camera_u0 = 375.0f;
float camera_v0 = 275.0f;
mat3 latest_rot = m_id3();
vec3 latest_pos = {0};

void callback_debug(downward_target_debug::debug msg)
{
    latest_rot = m_rotz(msg.rz)*m_roty(msg.ry)*m_rotx(msg.rx);
    latest_pos = m_vec3(msg.tx, msg.ty, msg.tz);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "downward_target_tracker");
    ros::NodeHandle node;
    ros::Publisher pub_image  = node.advertise<downward_target_tracker::image>("downward_target_tracker/image", 1);
    ros::Publisher pub_info   = node.advertise<downward_target_tracker::info>("downward_target_tracker/info", 1);
    ros::Publisher pub_tracks = node.advertise<downward_target_tracker::tracks>("downward_target_tracker/tracks", 1);
    ros::Subscriber sub_debug = node.subscribe("/downward_target_debug/debug", 1, callback_debug);

    signal(SIGINT, ctrlc);

    usbcam_opt_t opt = {0};
    opt.device_name = CAMERA_NAME;
    opt.pixel_format = V4L2_PIX_FMT_MJPEG;
    opt.width = CAMERA_WIDTH;
    opt.height = CAMERA_HEIGHT;
    opt.buffers = CAMERA_BUFFERS;
    usbcam_init(opt);

    const int Ix = CAMERA_WIDTH>>CAMERA_LEVELS;
    const int Iy = CAMERA_HEIGHT>>CAMERA_LEVELS;
    static unsigned char I[CAMERA_WIDTH*CAMERA_HEIGHT*3];

    uint64_t t_begin = get_nanoseconds();

    for (int frame = 0;; frame++)
    {
        unsigned char *jpg_data = 0;
        unsigned int jpg_size = 0;
        timeval timestamp = {0};
        usbcam_lock(&jpg_data, &jpg_size, &timestamp);

        float dt_frame = 0.0f;
        {
            uint64_t sec = (uint64_t)timestamp.tv_sec;
            uint64_t usec = (uint64_t)timestamp.tv_usec;
            uint64_t t = sec*1000*1000 + usec;
            static uint64_t last_t = t;
            dt_frame = (t-last_t)/1e6;
            last_t = t;
        }

        float dt_decompress = 0.0f;
        {
            uint64_t t1 = get_nanoseconds();
            if (!usbcam_jpeg_to_rgb(Ix, Iy, I, jpg_data, jpg_size))
            {
                usbcam_unlock();
                continue;
            }
            uint64_t t2 = get_nanoseconds();
            dt_decompress = (t2-t1)/1e9;
        }

        #if 0
        tracks_t tracks = {0};
        {
            track_targets_opt_t opt = {0};
            opt.platez = 0.1f;
            opt.merge_threshold = 0.3;
            opt.confidence_limit = 20;
            opt.initial_confidence = 5;
            opt.accept_confidence = 10;
            opt.removal_confidence = 0;
            opt.removal_time = 2.0f;
            opt.minimum_count = 50;
            opt.r_g = 3.0f;
            opt.r_b = 3.0f;
            opt.r_n = 10.0f/3.0f;
            opt.g_r = 1.6f;
            opt.g_b = 1.5f;
            opt.g_n = 10.0f/3.0f;
            opt.f = camera_f*Ix/CAMERA_WIDTH;
            opt.u0 = camera_u0*Ix/CAMERA_WIDTH;
            opt.v0 = camera_v0*Ix/CAMERA_WIDTH;
            opt.I = I;
            opt.Ix = Ix;
            opt.Iy = Iy;
            opt.rot = latest_rot;
            opt.pos = latest_pos;
            opt.gps = true;
            opt.timestamp = (get_nanoseconds()-t_begin)/1e9;
            tracks = track_targets(opt);
        }

        {
            downward_target_tracker::tracks msg;
            msg.count = tracks.count;
            for (int target = 0; target < tracks.count; target++)
            {
                msg.u.push_back(tracks.targets[target].last_seen.u);
                msg.v.push_back(tracks.targets[target].last_seen.v);
                msg.u1.push_back(tracks.targets[target].last_seen.u1);
                msg.v1.push_back(tracks.targets[target].last_seen.v1);
                msg.u2.push_back(tracks.targets[target].last_seen.u2);
                msg.v2.push_back(tracks.targets[target].last_seen.v2);
                msg.x_gps.push_back(tracks.targets[target].last_seen.x_gps);
                msg.y_gps.push_back(tracks.targets[target].last_seen.y_gps);
                msg.u_hat.push_back(tracks.targets[target].u_hat);
                msg.v_hat.push_back(tracks.targets[target].v_hat);
                msg.x_hat.push_back(tracks.targets[target].x_hat);
                msg.y_hat.push_back(tracks.targets[target].y_hat);
                msg.dx_hat.push_back(tracks.targets[target].dx_hat);
                msg.dy_hat.push_back(tracks.targets[target].dy_hat);
                msg.unique_id.push_back(tracks.targets[target].unique_id);
                msg.confidence.push_back(tracks.targets[target].confidence);
            }
            pub_tracks.publish(msg);
        }
        #endif

        {
            downward_target_tracker::info msg;
            msg.dt_frame = dt_frame;
            msg.dt_decompress = dt_decompress;
            msg.camera_f = camera_f;
            msg.camera_u0 = camera_u0;
            msg.camera_v0 = camera_v0;
            msg.camera_w = CAMERA_WIDTH;
            msg.camera_h = CAMERA_HEIGHT;
            pub_info.publish(msg);
        }

        {
            downward_target_tracker::image msg;
            msg.jpg_data.resize(jpg_size);
            memcpy(&msg.jpg_data[0], jpg_data, jpg_size);
            pub_image.publish(msg);
        }

        ros::spinOnce();

        usbcam_unlock();
    }

    return 0;
}
