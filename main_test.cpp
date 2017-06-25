#define debug_draw_input 0
#define debug_draw_calib 0
#define debug_draw_track 0
#define data_directory   "C:/Temp/data_3aug_2/"
#define poses_log_name   data_directory "log_poses.txt"
#define video_log_name   data_directory "log_video2.txt"
#define video_filename   data_directory "video_2_%04d.jpg"

#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include "vdb/vdb.h"

#define STB_IMAGE_IMPLEMENTATION
#include "asc_tracker.h"
#include "stb_image.h"

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
    float f_calibrated = 494.0f;
    float u0_calibrated = 649.0f;
    float v0_calibrated = 335.0f;
    float Ix_calibrated = 1280.0f;

    track_targets_opt_t opt = {0};
    opt.r_g = 3.0f;
    opt.r_b = 2.0f;
    opt.r_n = 10.0f/3.0f;
    opt.g_r = 1.6f;
    opt.g_b = 1.5f;
    opt.g_n = 10.0f/3.0f;

    const int max_log = 10000;
    int log_length = 0;

    // READ VIDEO LOG FILE
    static int   video_i[max_log]; // filename suffix
    static float video_t[max_log]; // timestamp
    {
        FILE *f = fopen(video_log_name, "r");
        assert(f);
        int i; // filename suffix
        uint64_t t; // timestamp in microseconds since UNIX epoch
        while (2 == fscanf(f, "%d %llu", &i, &t) && log_length < max_log)
        {
            static uint64_t t_begin = t;
            video_i[log_length] = i;
            video_t[log_length] = (float)(t-t_begin)/1e6;
            log_length++;
        }
        fclose(f);
    }

    // READ POSES LOG FILE
    static mat3 poses_rot[max_log]; // rotation corresponding to video_i
    static vec3 poses_pos[max_log]; // translation corresponding to video_i
    static bool poses_ok[max_log] = {0};
    {
        FILE *f = fopen(poses_log_name, "r");
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

    int skip_count = 1;
    for (int log_index = 1600; log_index < log_length; log_index+=skip_count)
    // for (int log_index = 0; log_index < log_length; log_index+=skip_count)
    {
        if (!poses_ok[video_i[log_index]])
            continue;

        // LOAD IMAGE
        int Ix, Iy, Ic;
        unsigned char *I;
        {
            char name[1024];
            sprintf(name, video_filename, video_i[log_index]);
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
        float f = f_calibrated*Ix/Ix_calibrated;
        float u0 = u0_calibrated*Ix/Ix_calibrated;
        float v0 = v0_calibrated*Ix/Ix_calibrated;

        tracks_t tracks = {0};
        {
            opt.f = f;
            opt.u0 = u0;
            opt.v0 = v0;
            opt.I = I;
            opt.Ix = Ix;
            opt.Iy = Iy;
            opt.rot = rot;
            opt.pos = pos;
            opt.timestamp = video_t[log_index];
            tracks = track_targets(opt);
        }

        #if debug_draw_input==1
        VDBB("Input");
        {
            vdbOrtho(-1.0f, +1.0f, +1.0f, -1.0f);
            vdbSetTexture2D(0, I, Ix, Iy, GL_RGB, GL_UNSIGNED_BYTE, GL_NEAREST, GL_NEAREST);
            vdbDrawTexture2D(0);
        }
        VDBE();
        #endif

        #if debug_draw_calib==1
        VDBB("Calibration");
        {
            vdbOrtho(-1.0f, +1.0f, +1.0f, -1.0f);
            vdbSetTexture2D(0, I, Ix, Iy, GL_RGB, GL_UNSIGNED_BYTE, GL_NEAREST, GL_NEAREST);
            vdbDrawTexture2D(0);
        }
        VDBE();
        #endif

        #if debug_draw_track==1
        VDBB("Tracks");
        {
            const int mode_draw_image = 0;
            const int mode_draw_world = 1;
            static int mode = mode_draw_image;

            SliderInt("skip_count", &skip_count, 1, 16);
            SliderInt("log_index", &log_index, 0, log_length-1);
            RadioButton("Draw image", &mode, mode_draw_image);
            RadioButton("Draw world", &mode, mode_draw_world);

            if (mode == mode_draw_image)
            {
                vdbOrtho(-1.0f, +1.0f, +1.0f, -1.0f);
                vdbSetTexture2D(0, I, Ix, Iy, GL_RGB, GL_UNSIGNED_BYTE, GL_NEAREST, GL_NEAREST);
                vdbDrawTexture2D(0);

                // draw grid
                {
                    int xw0 = (int)(pos.x);
                    int yw0 = (int)(pos.y);
                    vdbOrtho(0,Ix,Iy,0);
                    glLines(4.0f);
                    glColor4f(1.0f,1.0f,0.3f,1.0f);
                    for (int xw = xw0-5; xw <= xw0+5; xw++)
                    {
                        float u1,v1;
                        for (int i = 0; i < 128; i++)
                        {
                            float yw = yw0 - 5 + 10*i/128.0f;
                            vec3 p = { xw, yw, 0 };
                            vec3 q = m_transpose(rot)*(p - pos);
                            float x = q.x;
                            float y = q.y;
                            float z = q.z;
                            float u,v;
                            {
                                float l = sqrtf(x*x+y*y);
                                if (l < 0.001f)
                                {
                                    u = u0;
                                    v = v0;
                                }
                                else
                                {
                                    float t = atanf(-l/z);
                                    float r = f*t;
                                    u = u0 + r*x/l;
                                    v = v0 - r*y/l;
                                }
                            }
                            if (i > 0)
                            {
                                glVertex2f(u1,v1);
                                glVertex2f(u,v);
                            }
                            u1 = u;
                            v1 = v;
                        }
                    }
                    glEnd();
                }

                // draw connected components
                {
                    int *points = tracks.points;
                    int num_points = tracks.num_points;
                    cc_groups groups = tracks.groups;

                    int max_n = 0;
                    for (int i = 0; i < groups.count; i++)
                    {
                        if (groups.group_n[i] > max_n)
                            max_n = groups.group_n[i];
                    }

                    vdbOrtho(0.0f, Ix, Iy, 0.0f);
                    glBegin(GL_TRIANGLES);
                    for (int i = 0; i < num_points; i++)
                    {
                        int p = points[i];
                        int x = p % Ix;
                        int y = p / Ix;
                        int l = groups.label[p];
                        int n = groups.group_n[l];

                        if (n > 0.025f*max_n)
                        {
                            glColor4f(vdbPalette(l));
                            vdbFillRect(x, y, 1.0f, 1.0f);
                        }
                    }
                    glEnd();

                    vdbAdditiveBlend();
                    glLines(2.0f);
                    glColor4f(0.2f, 0.8f, 1.0f, 1.0f);
                    for (int i = 0; i < groups.count; i++)
                    {
                        if (groups.group_n[i] > 0.025f*max_n)
                        {
                            float min_x = groups.group_min_x[i];
                            float min_y = groups.group_min_y[i];
                            float max_x = groups.group_max_x[i];
                            float max_y = groups.group_max_y[i];
                            vdbDrawRect(min_x+0.5f, min_y+0.5f, max_x-min_x, max_y-min_y);
                        }
                    }
                    glEnd();
                    vdbAlphaBlend();

                    bool changed = false;
                    changed |= SliderFloat("r_g", &opt.r_g, 0.0f, 10.0f);
                    changed |= SliderFloat("r_b", &opt.r_b, 0.0f, 10.0f);
                    changed |= SliderFloat("r_n", &opt.r_n, 0.0f, 255.0f);
                    changed |= SliderFloat("g_r", &opt.g_r, 0.0f, 10.0f);
                    changed |= SliderFloat("g_b", &opt.g_b, 0.0f, 10.0f);
                    changed |= SliderFloat("g_n", &opt.g_n, 0.0f, 255.0f);
                    if (changed)
                    {
                        tracks = track_targets(opt);
                    }
                }

                // draw detections
                {
                    detection_t *detections = tracks.detections;
                    int num_detections = tracks.num_detections;
                    for (int i = 0; i < num_detections; i++)
                    {
                        float u = detections[i].u;
                        float v = detections[i].v;

                        glLines(2.0f);
                        glColor4f(1.0f,1.0f,0.2f, 1.0f);
                        vdbDrawCircle(u,v,6);
                        glEnd();
                    }
                }

                // draw tracks
                {
                    target_t *targets = tracks.targets;
                    int num_targets = tracks.num_targets;
                    for (int i = 0; i < num_targets; i++)
                    {
                        float u1 = targets[i].last_seen.u1;
                        float v1 = targets[i].last_seen.v1;
                        float u2 = targets[i].last_seen.u2;
                        float v2 = targets[i].last_seen.v2;
                        float u = targets[i].last_seen.u;
                        float v = targets[i].last_seen.v;

                        vdbNote(u,v,"ID: %d\nRate: %.2f",
                                targets[i].unique_id,
                                targets[i].detection_rate);
                    }
                }
            }

            // draw track windows
            if (mode == mode_draw_world)
            {
                float aspect = (float)vdb_input.width/vdb_input.height;
                vdbFreeSphereCamera();
                // vdbOrtho(-2.0f*aspect, +2.0f*aspect, -2.0f, +2.0f);
                glLines(2.0f);
                glColor4f(0.5f,0.5f,0.5f,1.0f);
                vdbGridXY(-2.0f, +2.0f, -2.0f, +2.0f, 4);
                glEnd();

                detection_t *detections = tracks.detections;
                int num_detections = tracks.num_detections;
                target_t *targets = tracks.targets;
                int num_targets = tracks.num_targets;
                for (int i = 0; i < num_targets; i++)
                {
                    // Draw velocity
                    float x = targets[i].last_seen.x;
                    float y = targets[i].last_seen.y;
                    float dx = targets[i].velocity_x;
                    float dy = targets[i].velocity_y;
                    glLines(1.0f);
                    glColor4f(1.0f, 1.0f, 0.2f, 1.0f);
                    glVertex2f(x, y);
                    glVertex2f(x+dx, y+dy);
                    glEnd();
                    vdbNote(x, y, "%.2f", sqrtf(dx*dx + dy*dy));

                    // Draw last seen position
                    glBegin(GL_TRIANGLES);
                    glColor4f(vdbPalette(i, 0.3f));
                    vdbFillCircle(targets[i].last_seen.x, targets[i].last_seen.y, 0.15f);
                    glEnd();

                    // Draw history of detections
                    glLines(2.0f);
                    glColor4f(vdbPalette(i));
                    for (int j = 0; j < targets[i].num_window-1; j++)
                    {
                        float x1 = targets[i].window[j].x;
                        float y1 = targets[i].window[j].y;
                        float x2 = targets[i].window[j+1].x;
                        float y2 = targets[i].window[j+1].y;
                        glVertex2f(x1,y1);
                        glVertex2f(x2,y2);
                    }
                    glEnd();
                }
            }

            if (vdbKeyDown(SPACE))
                vdbStepOnce();
        }
        VDBE();
        #endif

        free(I);
    }
    return 0;
}
