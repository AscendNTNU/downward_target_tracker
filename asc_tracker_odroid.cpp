//
// notes for myself
//
// R_cam^imu = Rz(cam_imu_rz)Ry(cam_imu_ry)Rx(cam_imu_rx) [camera to imu frame]
// T_cam/imu^imu = (cam_imu_tx, cam_imu_ty, cam_imu_tz)   [camera relative imu in imu frame]

//
// below are compile-time settings
//

#define mock_image         1
#define disable_ros        1

#if mock_image==0
#define USBCAM_DEBUG       1
#define device_name        "/dev/video1"
#define camera_width       800
#define camera_height      600
#define camera_buffers     3
#define camera_levels      2 // Downscale factor (0=none, 1=half, 2=quarter)
#define camera_f_init      434.0f
#define camera_u0_init     375.0f
#define camera_v0_init     275.0f
#else
#include "mock_jpg.h"      // hint: run "xxd -i" on an image to generate a header-embedded binary
#define camera_width       1280
#define camera_height      720
#define camera_levels      3
#define camera_f_init      494.0f
#define camera_u0_init     649.0f
#define camera_v0_init     335.0f
#endif

#define cam_imu_rx_init    0.0f
#define cam_imu_ry_init    0.0f
#define cam_imu_rz_init    0.0f
#define cam_imu_tx_init    0.0f
#define cam_imu_ty_init    0.0f
#define cam_imu_tz_init    0.0f
#define r_g_init           3.0f
#define r_b_init           2.0f
#define r_n_init           10.0f/3.0f
#define g_r_init           1.6f
#define g_b_init           1.5f
#define g_n_init           10.0f/3.0f

//
// below is lots of code
//

#include <signal.h>
#include <assert.h>
#include <stdint.h>
#if disable_ros==0
#include <ros/ros.h>
#include <downward_target_tracker/image.h>
#include <downward_target_tracker/info.h>
#include <downward_target_tracker/tracks.h>
#include <downward_target_debug/debug.h>
#endif
#include "vdb_release.h"
#include "asc_usbcam.h"
#include "asc_tracker.h"
#include "get_nanoseconds.h"

//
// OPTIONS
//
float camera_f = camera_f_init;
float camera_u0 = camera_u0_init;
float camera_v0 = camera_v0_init;
float cam_imu_rx = cam_imu_rx_init;
float cam_imu_ry = cam_imu_ry_init;
float cam_imu_rz = cam_imu_rz_init;
float cam_imu_tx = cam_imu_tx_init;
float cam_imu_ty = cam_imu_ty_init;
float cam_imu_tz = cam_imu_tz_init;
float r_g = r_g_init;
float r_b = r_b_init;
float r_n = r_n_init;
float g_r = g_r_init;
float g_b = g_b_init;
float g_n = g_n_init;

//
// LATEST MESSAGES
//
mat3 latest_imu_rot = m_id3();
vec3 latest_imu_pos = m_vec3(0.0f, 0.0f, 1.0f);

#if disable_ros==0
void callback_debug(downward_target_debug::debug msg)
{

}
#endif

void ctrlc(int)
{
    exit(0);
}

int main(int argc, char **argv)
{
    #if disable_ros==0
    ros::init(argc, argv, "downward_target_tracker");
    ros::NodeHandle node;
    ros::Publisher pub_image  = node.advertise<downward_target_tracker::image>("downward_target_tracker/image", 1);
    ros::Publisher pub_info   = node.advertise<downward_target_tracker::info>("downward_target_tracker/info", 1);
    ros::Publisher pub_tracks = node.advertise<downward_target_tracker::tracks>("downward_target_tracker/tracks", 1);
    ros::Subscriber sub_debug = node.subscribe("/downward_target_debug/debug", 1, callback_debug);
    #endif

    signal(SIGINT, ctrlc);

    #if mock_image==0
    {
        usbcam_opt_t opt = {0};
        opt.device_name = device_name;
        opt.pixel_format = V4L2_PIX_FMT_MJPEG;
        opt.width = camera_width;
        opt.height = camera_height;
        opt.buffers = camera_buffers;
        usbcam_init(opt);
    }
    #endif

    const int Ix = camera_width>>camera_levels;
    const int Iy = camera_height>>camera_levels;
    static unsigned char I[camera_width*camera_height*3];
    uint64_t t_begin = get_nanoseconds();
    for (int frame = 0;; frame++)
    {
        unsigned char *jpg_data = 0;
        unsigned int jpg_size = 0;
        timeval timestamp = {0};
        #if mock_image==1
        jpg_data = mock_jpg;
        jpg_size = mock_jpg_len;
        usleep(16*1000);
        #else
        usbcam_lock(&jpg_data, &jpg_size, &timestamp);
        #endif

        float dt_frame = 0.0f;
        {
            uint64_t sec = (uint64_t)timestamp.tv_sec;
            uint64_t usec = (uint64_t)timestamp.tv_usec;
            uint64_t t = sec*1000*1000 + usec;
            static uint64_t last_t = t;
            dt_frame = (t-last_t)/1e6;
            last_t = t;
        }

        float dt_jpeg_to_rgb = 0.0f;
        {
            uint64_t t1 = get_nanoseconds();
            if (!usbcam_jpeg_to_rgb(Ix, Iy, I, jpg_data, jpg_size))
            {
                usbcam_unlock();
                continue;
            }
            uint64_t t2 = get_nanoseconds();
            dt_jpeg_to_rgb = (t2-t1)/1e9;
        }

        #if disable_ros==0
        ros::spinOnce();
        #endif

        mat3 cam_imu_rot = m_rotz(cam_imu_rz)*m_roty(cam_imu_ry)*m_rotx(cam_imu_rx);
        vec3 cam_imu_pos = m_vec3(cam_imu_tx, cam_imu_ty, cam_imu_tz);
        mat3 rot = latest_imu_rot*cam_imu_rot;
        vec3 pos = latest_imu_pos + latest_imu_rot*cam_imu_pos;
        float f = camera_f*Ix/camera_width;
        float u0 = camera_u0*Ix/camera_width;
        float v0 = camera_v0*Ix/camera_width;

        tracks_t tracks = {0};
        float dt_track_targets = 0.0f;
        {
            uint64_t t1 = get_nanoseconds();
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
            opt.gps = true;
            opt.timestamp = (get_nanoseconds()-t_begin)/1e9;
            tracks = track_targets(opt);
            uint64_t t2 = get_nanoseconds();
            dt_track_targets = (t2-t1)/1e9;
        }

        #if disable_ros==1

        // COLOR SEGMENTATION TEST
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

        // TARGET TRACKING TEST
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

        // CALIBRATION TEST
        #if 1
        if (vdb_begin())
        {
            vdb_xrange(-4.0f, +4.0f);
            vdb_yrange(-2.0f, +2.0f);
            for (int y = 1; y < Iy-1; y++)
            for (int x = 1; x < Ix-1; x++)
            {
                vec2 uv = { x+0.5f, y+0.5f };
                vec3 dir = rot*camera_inverse_project(f,u0,v0, uv);
                vec2 p;
                if (m_intersect_xy_plane(dir, pos.z, &p))
                {
                    vdb_color_rampf(I[3*(x + y*Ix)+0]/255.0f);
                    vdb_point(p.x, p.y);
                }
            }
            vdb_end();
        }
        #endif
        #endif

        #if disable_ros==0
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
            msg.dt_jpeg_to_rgb = dt_jpeg_to_rgb;
            msg.dt_track_targets = dt_track_targets;
            msg.camera_f = camera_f;
            msg.camera_u0 = camera_u0;
            msg.camera_v0 = camera_v0;
            msg.camera_w = camera_width;
            msg.camera_h = camera_height;
            pub_info.publish(msg);
        }

        {
            downward_target_tracker::image msg;
            msg.jpg_data.resize(jpg_size);
            memcpy(&msg.jpg_data[0], jpg_data, jpg_size);
            pub_image.publish(msg);
        }
        #endif

        #if mock_image==0
        usbcam_unlock();
        #endif
    }

    return 0;
}
