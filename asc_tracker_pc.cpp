#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <vdb.h>

#define STB_IMAGE_IMPLEMENTATION
#include "asc_tracker.h"
#include "stb_image.h"

#define DATA_DIR            "C:/Temp/data_3aug_2/"
#define VIDEO_LOG  DATA_DIR "log_video2.txt"
#define POSES_LOG  DATA_DIR "log_poses.txt"
#define VIDEO_NAME DATA_DIR "video_2_%04d.jpg"
#define MAX_LOGS 10000

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
    int levels = 3;

    float rotz_camera_body = -3.1415f/2.0f;
    float f_calibrated = 494.0f;
    float u0_calibrated = 649.0f;
    float v0_calibrated = 335.0f;
    float downscale_factor = 1.0f / (float)(1 << levels);
    float f = f_calibrated*downscale_factor;
    float u0 = u0_calibrated*downscale_factor;
    float v0 = v0_calibrated*downscale_factor;

    const int max_log = 10000;
    int log_length = 0;

    // READ VIDEO LOG FILE
    static int   video_i[max_log]; // filename suffix
    static float video_t[max_log]; // timestamp
    {
        FILE *f = fopen(DATA_DIR "log_video2.txt", "r");
        assert(f);
        int i; // filename suffix
        uint64_t t; // timestamp in microseconds since UNIX epoch
        while (2 == fscanf(f, "%d %llu", &i, &t) && log_length < max_log)
        {
            video_i[log_length] = i;
            video_t[log_length] = t/1e6;
            log_length++;
        }
        fclose(f);
    }

    // READ POSES LOG FILE
    static mat3 poses_rot[max_log]; // rotation corresponding to video_i
    static vec3 poses_pos[max_log]; // translation corresponding to video_i
    static bool poses_ok[max_log] = {0};
    {
        FILE *f = fopen(POSES_LOG, "r");
        assert(f);
        int i; // corresponding video suffix
        float rx, ry, rz, tx, ty, tz; // rotation and translation
        while (7 == fscanf(f, "%d %f %f %f %f %f %f", &i, &rx,&ry,&rz, &tx,&ty,&tz))
        {
            poses_rot[i] = m_rotz(rz)*m_roty(ry)*m_rotx(rx);
            poses_pos[i] = m_vec3(tx, ty, tz);
            poses_ok[i] = true;
        }
        fclose(f);
    }

    for (int log_index = 0; log_index < log_length; log_index++)
    {
        if (!poses_ok[video_i[log_index]])
            continue;

        // LOAD IMAGE
        int Ix, Iy, Ic;
        unsigned char *I;
        {
            char name[1024];
            sprintf(name, VIDEO_NAME, video_i[log_index]);
            I = stbi_load(name, &Ix, &Iy, &Ic, 3);
            assert(I);
        }

        // RESIZE IMAGE
        for (int i = 0; i < levels; i++)
        {
            downsample(I, Ix, Iy);
            Ix /= 2;
            Iy /= 2;
        }

        mat3 rot = poses_rot[video_i[log_index]];
        vec3 pos = poses_pos[video_i[log_index]];

        VDBB("Input");
        {
            vdbOrtho(-1.0f, +1.0f, +1.0f, -1.0f);
            vdbSetTexture2D(0, I, Ix, Iy, GL_RGB, GL_UNSIGNED_BYTE, GL_NEAREST, GL_NEAREST);
            vdbDrawTexture2D(0);
        }
        VDBE();

        free(I);
    }

    #if 0

    read_log_data();
    float timestamp = 0.0f;
    for (int log_index = 1000; log_index < log_count; log_index++)
    {
        TIMING("load");
        int Ix, Iy, Ic;
        char name[1024];
        sprintf(name, VIDEO_NAME, log_entries[log_index].video_i);
        unsigned char *I = stbi_load(name, &Ix, &Iy, &Ic, 3);
        assert(I);
        TIMING("load");

        TIMING("resize");
        for (int i = 0; i < levels; i++)
        {
            downsample(I, Ix, Iy);
            Ix /= 2;
            Iy /= 2;
        }
        TIMING("resize");

        mat3 rot;
        vec3 pos;
        static vec3 last_gps_pos;

        if (gps_ok[log_index])
        {
            rot = gps_rot[log_index];
            pos = gps_pos[log_index];
            last_gps_pos = pos;
        }
        else
        {
            vec4 q_body_grid = {0};
            q_body_grid.x = log_entries[log_index].qx;
            q_body_grid.y = log_entries[log_index].qy;
            q_body_grid.z = log_entries[log_index].qz;
            q_body_grid.w = log_entries[log_index].qw;
            mat3 rot_body_grid = m_quat_to_so3(q_body_grid); // R_b^g
            mat3 rot_camera_body = m_rotz(rotz_camera_body); // R_c^b
            rot = rot_body_grid*rot_camera_body;             // R_c^g = R_b^g * R_c^b
            pos.x = last_gps_pos.x;
            pos.y = last_gps_pos.y;
            pos.z = m_dot(log_entries[log_index].r*rot.a3, m_vec3(0,0,1));
        }

        #if 0 // DEBUG GROUND PLANE PROJECTION
        if (vdb_begin())
        {
            static int threshold = 20;
            vdb_xrange(-4.0f, +4.0f);
            vdb_yrange(-2.0f, +2.0f);
            for (int y = 1; y < Iy-1; y += 2)
            for (int x = 1; x < Ix-1; x += 2)
            {
                vec2 uv = { x+0.5f, y+0.5f };
                vec3 dir = rot*m_ray_equidistant(f,u0,v0, uv);
                vec2 p;
                if (m_intersect_xy_plane(dir, camz, &p))
                {
                    vdb_color_rampf(I[3*(x + y*Ix)+0]/255.0f);
                    vdb_point(p.x, p.y);
                }
            }
            vdb_slider1i("threshold", &threshold, 0, 200);
            vdb_end();
        }

        #else
        TIMING("track");
        track_targets(I,Ix,Iy, f,u0,v0, rot, pos, gps_ok[log_index], timestamp);
        TIMING("track");
        #endif

        TIMING_SUMMARY();

        timestamp += 1.0f/60.0f;
        free(I);
    }
    #endif
    return 0;
}
