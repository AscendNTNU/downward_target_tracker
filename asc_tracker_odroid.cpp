#define TEST_DECOMPRESSION
// #define TEST_CALIBRATION
#define ENABLE_TIMING

#include "asc_profiler.h"
#include "asc_usbcam.h"
#include <signal.h>
#include <assert.h>
#include <stdint.h>

uint64_t get_nanoseconds()
// Returns number of nanoseconds since the UNIX epoch
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
    uint64_t dt = t-last_t;
    if (dt > required_dt*1e9)
    {
        return vdb_begin();
    }
    else
    {
        return 0;
    }
}

void ctrlc(int)
{
    usb_shutdown();
}

void downsample(unsigned char *src, int w, int h)
{
    assert(src != 0);
    assert(w % 2 == 0);
    assert(h % 2 == 0);
    int dw = w/2;
    int dh = h/2;
    for (int y = 0; y < dh; y++)
    for (int x = 0; x < dw; x++)
    {
        int x1 = x*2;
        int x2 = x*2+1;
        int y1 = y*2;
        int y2 = y*2+1;
        unsigned char r11 = src[(y1*w + x1)*3+0] >> 2;
        unsigned char r12 = src[(y2*w + x1)*3+0] >> 2;
        unsigned char r21 = src[(y1*w + x2)*3+0] >> 2;
        unsigned char r22 = src[(y2*w + x2)*3+0] >> 2;
        unsigned char g11 = src[(y1*w + x1)*3+1] >> 2;
        unsigned char g12 = src[(y2*w + x1)*3+1] >> 2;
        unsigned char g21 = src[(y1*w + x2)*3+1] >> 2;
        unsigned char g22 = src[(y2*w + x2)*3+1] >> 2;
        unsigned char b11 = src[(y1*w + x1)*3+2] >> 2;
        unsigned char b12 = src[(y2*w + x1)*3+2] >> 2;
        unsigned char b21 = src[(y1*w + x2)*3+2] >> 2;
        unsigned char b22 = src[(y2*w + x2)*3+2] >> 2;
        src[(y*dw + x)*3+0] = (r11+r12+r21+r22);
        src[(y*dw + x)*3+1] = (g11+g12+g21+g22);
        src[(y*dw + x)*3+2] = (b11+b12+b21+b22);
    }
}

int main(int, char **)
{
    signal(SIGINT, ctrlc);

    const int Ix0 = 800;
    const int Iy0 = 600;
    usb_init(Ix0, Iy0, "/dev/video0", 60, 1);
    unsigned char I[Ix0*Iy0*3];

    float camera_f = 434.0f;
    float camera_u0 = 375.0f;
    float camera_v0 = 275.0f;

    while (usb_recvFrame(I))
    {
        TIMING("downsample");
        int Ix = Ix0;
        int Iy = Iy0;
        downsample(I, Ix, Iy); Ix /= 2; Iy /= 2;
        downsample(I, Ix, Iy); Ix /= 2; Iy /= 2;
        TIMING("downsample");

        TIMING("vdb");
        #if defined(TEST_DECOMPRESSION)
        if (vdb_begin(0.1f))
        {
            vdb_imageRGB8(I, Ix, Iy);
            vdb_end();
        }
        #elif defined(TEST_CALIBRATION)
        if (vdb_begin(0.1f))
        {
            static float camera_ex = 0.0f;
            static float camera_ey = 0.0f;
            static float camera_ez = 0.0f;
            static float camera_z = 0.0f;
            static int threshold = 20;

            mat3 camera_to_world = m_rotz(camera_ez)*m_roty(camera_ey)*m_rotx(camera_ex);
            float f = camera_f * Ix/Ix0;
            float u0 = camera_u0 * Ix/Ix0;
            float v0 = camera_v0 * Ix/Ix0;

            vdb_xrange(-4.0f, +4.0f);
            vdb_yrange(-2.0f, +2.0f);
            for (int y = 1; y < Iy-1; y += 2)
            for (int x = 1; x < Ix-1; x += 2)
            {
                vec2 uv = { x+0.5f, y+0.5f };
                vec3 dir = camera_to_world*m_ray_equidistant(f,u0,v0, uv);
                vec2 p;
                if (m_intersect_xy_plane(dir, camera_z, &p))
                {
                    vdb_color_rampf(I[3*(x + y*Ix)+0]/255.0f);
                    vdb_point(p.x, p.y);
                }
            }
            vdb_slider1i("threshold", &threshold, 0, 200);
            vdb_slider1f("camera_f", &camera_f, 400.0f, 500.0f);
            vdb_slider1f("camera_u0", &camera_u0, 0.0f, Ix);
            vdb_slider1f("camera_v0", &camera_v0, 0.0f, Iy);
            vdb_slider1f("camera_z", &camera_z, 0.0f, 1.0f);
            vdb_slider1f("camera_ex", &camera_ex, -0.3f, +0.3f);
            vdb_slider1f("camera_ey", &camera_ey, -0.3f, +0.3f);
            vdb_slider1f("camera_ez", &camera_ez, -3.14f, +3.14f);
            vdb_end();
        }
        #endif
        TIMING("vdb");

        TIMING_SUMMARY();
    }

    usb_shutdown();
    return 0;
}
