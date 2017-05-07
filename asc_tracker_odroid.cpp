#define USBCAM_DEBUG
#define CAMERA_NAME        "/dev/video1"
#define CAMERA_WIDTH       800
#define CAMERA_HEIGHT      600
#define CAMERA_BUFFERS     3
#define CAMERA_LEVELS      2 // Downscale factor (0=none, 1=half, 2=quarter)
#define DISABLE_ROS        1

#include <signal.h>
#include <assert.h>
#include <stdint.h>

#if DISABLE_ROS==0
#include <ros/ros.h>
#include <downward_target_tracker/image.h>
#include <downward_target_tracker/info.h>
#include <downward_target_tracker/tracks.h>
#include <downward_target_debug/debug.h>
#endif

#if DISABLE_ROS==1
#include "vdb_release.h"
#endif
#include "asc_usbcam.h"
#include "asc_tracker.h"
#include "get_nanoseconds.h"

// OPTIONS
float camera_f = 434.0f;
float camera_u0 = 375.0f;
float camera_v0 = 275.0f;
float r_g = 3.0f;
float r_b = 2.0f;
float r_n = 10.0f/3.0f;
float g_r = 1.6f;
float g_b = 1.5f;
float g_n = 10.0f/3.0f;

mat3 latest_rot = m_id3();
vec3 latest_pos = {0};

#if DISABLE_ROS==0
void callback_debug(downward_target_debug::debug msg)
{
    latest_rot = m_rotz(msg.rz)*m_roty(msg.ry)*m_rotx(msg.rx);
    latest_pos = m_vec3(msg.tx, msg.ty, msg.tz);
}
#endif

void ctrlc(int)
{
    exit(0);
}

int main(int argc, char **argv)
{
    #if DISABLE_ROS==0
    ros::init(argc, argv, "downward_target_tracker");
    ros::NodeHandle node;
    ros::Publisher pub_image  = node.advertise<downward_target_tracker::image>("downward_target_tracker/image", 1);
    ros::Publisher pub_info   = node.advertise<downward_target_tracker::info>("downward_target_tracker/info", 1);
    ros::Publisher pub_tracks = node.advertise<downward_target_tracker::tracks>("downward_target_tracker/tracks", 1);
    ros::Subscriber sub_debug = node.subscribe("/downward_target_debug/debug", 1, callback_debug);
    #endif

    signal(SIGINT, ctrlc);

    {
        usbcam_opt_t opt = {0};
        opt.device_name = CAMERA_NAME;
        opt.pixel_format = V4L2_PIX_FMT_MJPEG;
        opt.width = CAMERA_WIDTH;
        opt.height = CAMERA_HEIGHT;
        opt.buffers = CAMERA_BUFFERS;
        usbcam_init(opt);
    }

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

        {
            latest_rot = m_id3();
            latest_pos = m_vec3(0.0f, 0.0f, 1.0f);
        }

        tracks_t tracks = {0};
        {
            track_targets_opt_t opt = {0};
            opt.r_g = r_g;
            opt.r_b = r_b;
            opt.r_n = r_n;
            opt.g_r = g_r;
            opt.g_b = g_b;
            opt.g_n = g_n;
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

        #if DISABLE_ROS==1
        #if 0
        if (vdb_begin())
        {
            cc_groups groups = tracks.groups;
            int *points = tracks.points;
            int num_points = tracks.num_points;

            int max_n = 0;
            for (int i = 0; i < groups.count; i++)
            {
                if (groups.group_n[i] > max_n)
                    max_n = groups.group_n[i];
            }
            vdb_setNicePoints(0);
            vdb_imageRGB8(I, Ix, Iy);

            vdb_xrange(0.0f, Ix);
            vdb_yrange(0.0f, Iy);

            vdb_translucent();
            vdb_color_white(2);
            vdb_fillRect(0.0f, 0.0f, Ix, Iy);

            vdb_opaque();
            vdb_color_red(1);
            for (int i = 0; i < num_points; i++)
            {
                int p = points[i];
                int x = p % Ix;
                int y = p / Ix;
                int l = groups.label[p];
                int n = groups.group_n[l];

                if (n > 0.025f*max_n)
                {
                    vdb_fillRect(x, y, 1.0f, 1.0f);
                }
            }

            vdb_color_black(2);
            for (int i = 0; i < groups.count; i++)
            {
                if (groups.group_n[i] > 0.025f*max_n)
                {
                    float min_x = groups.group_min_x[i];
                    float min_y = groups.group_min_y[i];
                    float max_x = groups.group_max_x[i];
                    float max_y = groups.group_max_y[i];
                    vdb_line(min_x, min_y, max_x, min_y);
                    vdb_line(max_x, min_y, max_x, max_y);
                    vdb_line(max_x, max_y, min_x, max_y);
                    vdb_line(min_x, max_y, min_x, min_y);
                }
            }

            {
                vdb_slider1f("r_g", &r_g, 0.0f, 5.0f);
                vdb_slider1f("r_b", &r_b, 0.0f, 5.0f);
                vdb_slider1f("r_n", &r_n, 0.0f, 5.0f);
                vdb_slider1f("g_r", &g_r, 0.0f, 5.0f);
                vdb_slider1f("g_b", &g_b, 0.0f, 5.0f);
                vdb_slider1f("g_n", &g_n, 0.0f, 5.0f);
            }

            vdb_end();
        }
        #endif

        #if 0
        if (vdb_begin())
        {
            int num_targets = tracks.num_targets;
            target_t *targets = tracks.targets;

            static int selected_id = -1;

            vdb_setNicePoints(1);
            vdb_imageRGB8(I, Ix, Iy);

            vdb_xrange(0.0f, (float)Ix);
            vdb_yrange(0.0f, (float)Iy);

            for (int i = 0; i < num_targets; i++)
            {
                float u1 = targets[i].last_seen.u1;
                float v1 = targets[i].last_seen.v1;
                float u2 = targets[i].last_seen.u2;
                float v2 = targets[i].last_seen.v2;
                float u = targets[i].u_hat;
                float v = targets[i].v_hat;

                float mx,my;
                if (vdb_mouse_click(&mx, &my))
                {
                    if (mx >= u1 && mx <= u2 && my >= v1 && my <= v2)
                    {
                        selected_id = targets[i].unique_id;
                    }
                }

                if (targets[i].unique_id == selected_id)
                    vdb_color_red(2);
                else
                    vdb_color_white(0);

                vdb_line(u1,v1, u2,v1);
                vdb_line(u2,v1, u2,v2);
                vdb_line(u2,v2, u1,v2);
                vdb_line(u1,v2, u1,v1);
                vdb_point(u, v);
            }
            vdb_end();
        }
        #endif
        #endif

        #if DISABLE_ROS==0

        {
            downward_target_tracker::tracks msg;
            msg.num_targets = tracks.num_targets;
            // msg.num_detections = tracks.num_detections;
            for (int i = 0; i < tracks.num_detections; i++)
            {
                msg.u.push_back(tracks.detections[i].u);
                msg.v.push_back(tracks.detections[i].v);
                msg.u1.push_back(tracks.detections[i].u1);
                msg.v1.push_back(tracks.detections[i].v1);
                msg.u2.push_back(tracks.detections[i].u2);
                msg.v2.push_back(tracks.detections[i].v2);

                // msg.u.push_back(tracks.targets[i].last_seen.u);
                // msg.v.push_back(tracks.targets[i].last_seen.v);
                // msg.u1.push_back(tracks.targets[i].last_seen.u1);
                // msg.v1.push_back(tracks.targets[i].last_seen.v1);
                // msg.u2.push_back(tracks.targets[i].last_seen.u2);
                // msg.v2.push_back(tracks.targets[i].last_seen.v2);
                // msg.x_gps.push_back(tracks.targets[i].last_seen.x_gps);
                // msg.y_gps.push_back(tracks.targets[i].last_seen.y_gps);
                // msg.u_hat.push_back(tracks.targets[i].u_hat);
                // msg.v_hat.push_back(tracks.targets[i].v_hat);
                // msg.x_hat.push_back(tracks.targets[i].x_hat);
                // msg.y_hat.push_back(tracks.targets[i].y_hat);
                // msg.dx_hat.push_back(tracks.targets[i].dx_hat);
                // msg.dy_hat.push_back(tracks.targets[i].dy_hat);
                // msg.unique_id.push_back(tracks.targets[i].unique_id);
                // msg.confidence.push_back(tracks.targets[i].confidence);
            }
            pub_tracks.publish(msg);
        }

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
        #endif

        usbcam_unlock();
    }

    return 0;
}
