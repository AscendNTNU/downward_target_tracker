// todo
//
// x Verify 800x600 calibration
// x Verify turbojpeg downscaling
// o Height cutoff
// o Jump acceptance test
// o ROS optitrack input
// o ROS debug io with mission debugger
//   o image
//   o tracks
//   o camera calibration
//   o timing info (vdb)
//   o timing info (jpeg)
//   o timing info (track)

// compile
//   $ g++ asc_tracker_odroid.cpp -L/usr/local/lib64 -o app -lv4l2 -lturbojpeg

#define USBCAM_DEBUG
#define CAMERA_NAME        "/dev/video1"
#define CAMERA_WIDTH       800
#define CAMERA_HEIGHT      600
#define CAMERA_BUFFERS     3
#define CAMERA_LEVELS      2 // Downscale factor (0=none, 1=half, 2=quarter)
#define TEST_DECOMPRESSION 0
#define TEST_CALIBRATION   1

#include "asc_usbcam.h"
#include "vdb_release.h"
#include "asc_tracker.h"
#include <time.h>
#include <signal.h>
#include <assert.h>
#include <stdint.h>

uint64_t get_nanoseconds()
{
    struct timespec ts = {};
    clock_gettime(CLOCK_REALTIME, &ts);
    uint64_t result = ((uint64_t)ts.tv_sec)*1000000000 +
                      ((uint64_t)ts.tv_nsec);
    return result;
}

int vdb_begin(float required_dt)
{
    static uint64_t last_t = get_nanoseconds();
    uint64_t t = get_nanoseconds();
    float dt = (t-last_t)/1e9;
    if (dt > required_dt)
    {
        if (vdb_begin())
        {
            last_t = t;
            return 1;
        }
    }
    return 0;
}

void ctrlc(int)
{
    exit(0);
}

int main(int, char **)
{
    signal(SIGINT, ctrlc);

    usbcam_opt_t opt = {0};
    opt.device_name = CAMERA_NAME;
    opt.pixel_format = V4L2_PIX_FMT_MJPEG;
    opt.width = CAMERA_WIDTH;
    opt.height = CAMERA_HEIGHT;
    opt.buffers = CAMERA_BUFFERS;
    usbcam_init(opt);

    float camera_f = 434.0f;
    float camera_u0 = 375.0f;
    float camera_v0 = 275.0f;

    const int Ix = CAMERA_WIDTH>>CAMERA_LEVELS;
    const int Iy = CAMERA_HEIGHT>>CAMERA_LEVELS;
    static unsigned char I[Ix*Iy*3];

    for (int i = 0;; i++)
    {
        unsigned char *jpg_data = 0;
        unsigned int jpg_size = 0;
        timeval timestamp = {0};
        usbcam_lock(&jpg_data, &jpg_size, &timestamp);

        float dt_internal;
        {
            uint64_t sec = (uint64_t)timestamp.tv_sec;
            uint64_t usec = (uint64_t)timestamp.tv_usec;
            uint64_t t = sec*1000*1000 + usec;
            static uint64_t last_t = t;
            dt_internal = (t-last_t)/1e6;
            last_t = t;
        }

        float dt_decompress;
        {
            uint64_t t1 = get_nanoseconds();
            if (!usbcam_mjpeg_to_rgb(Ix, Iy, I, jpg_data, jpg_size))
                continue;
            uint64_t t2 = get_nanoseconds();
            dt_decompress = (t2-t1)/1e9;
        }

        #if 1
        if (vdb_begin(0.1f))
        {
            #if TEST_DECOMPRESSION==1
            vdb_imageRGB8(I, Ix, Iy);
            #endif

            #if TEST_CALIBRATION==1
            static float camera_ex = 0.0f;
            static float camera_ey = 0.0f;
            static float camera_ez = 0.0f;
            static float camera_z = 1.0f;
            static int threshold = 20;

            mat3 camera_to_world = m_rotz(camera_ez)*m_roty(camera_ey)*m_rotx(camera_ex);
            float f = camera_f * Ix/CAMERA_WIDTH;
            float u0 = camera_u0 * Ix/CAMERA_WIDTH;
            float v0 = camera_v0 * Ix/CAMERA_WIDTH;

            vdb_xrange(-4.0f, +4.0f);
            vdb_yrange(-2.0f, +2.0f);
            for (int y = 1; y < Iy-1; y += 8)
            for (int x = 1; x < Ix-1; x += 8)
            {
                vec2 uv = { x+0.5f, y+0.5f };
                vec3 dir = camera_to_world*camera_inverse_project(f,u0,v0, uv);
                vec2 p;
                if (m_intersect_xy_plane(dir, camera_z, &p))
                {
                    vdb_color_rampf(I[3*(x + y*Ix)+0]/255.0f);
                    vdb_point(p.x, p.y);
                }
            }
            vdb_slider1i("threshold", &threshold, 0, 200);
            vdb_slider1f("camera_f", &camera_f, 400.0f, 500.0f);
            vdb_slider1f("camera_u0", &camera_u0, 0.0f, CAMERA_WIDTH);
            vdb_slider1f("camera_v0", &camera_v0, 0.0f, CAMERA_HEIGHT);
            vdb_slider1f("camera_z", &camera_z, 0.0f, 3.0f);
            vdb_slider1f("camera_ex", &camera_ex, -0.3f, +0.3f);
            vdb_slider1f("camera_ey", &camera_ey, -0.3f, +0.3f);
            vdb_slider1f("camera_ez", &camera_ez, -3.14f, +3.14f);
            #endif

            vdb_end();
        }
        #endif

        printf("%6.2f ms (decompress)\t%6.2f ms (frame)\n", 1000.0f*dt_decompress, 1000.0f*dt_internal);
        usbcam_unlock();
    }

    return 0;
}
