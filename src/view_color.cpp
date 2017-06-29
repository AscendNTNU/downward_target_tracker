#include "asc_detector.h"

void view_color(latest_image_t latest_image, downward_target_tracker::info latest_info)
{
    unsigned char *I = latest_image.I;
    int Ix = latest_image.Ix;
    int Iy = latest_image.Iy;
    float r_g = latest_info.r_g;
    float r_b = latest_info.r_b;
    float r_n = latest_info.r_n;
    float g_r = latest_info.g_r;
    float g_b = latest_info.g_b;
    float g_n = latest_info.g_n;
    vdbOrtho(-1.0f, +1.0f, +1.0f, -1.0f);
    vdbSetTexture2D(0, I, Ix, Iy, GL_RGB, GL_UNSIGNED_BYTE, GL_NEAREST, GL_NEAREST);
    vdbDrawTexture2D(0);

    // Find connected components belonging of top plate sections
    int *points;
    int num_points;
    cc_groups groups;
    cc_options opt = {0};
    opt.r_g = r_g;
    opt.r_b = r_b;
    opt.r_n = r_n;
    opt.g_r = g_r;
    opt.g_b = g_b;
    opt.g_n = g_n;
    groups = cc_find_top_plates(I, Ix, Iy, opt, &points, &num_points);

    // draw connected components
    {
        #if 0
        int max_n = 0;
        for (int i = 0; i < groups.count; i++)
        {
            if (groups.group_n[i] > max_n)
                max_n = groups.group_n[i];
        }
        #endif

        vdbOrtho(0.0f, Ix, Iy, 0.0f);
        glBegin(GL_TRIANGLES);
        for (int i = 0; i < num_points; i++)
        {
            int p = points[i];
            int x = p % Ix;
            int y = p / Ix;
            int l = groups.label[p];
            int n = groups.group_n[l];

            #if 0
            if (n > 0.025f*max_n)
            #endif
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
            #if 0
            // if (groups.group_n[i] > 0.025f*max_n)
            #endif
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
    }

    {
        using namespace ImGui;
        Begin("Readme (color calibration)");
        TextWrapped(
            "1. Keep a red and green target plate in view.\n"
            "2. Take snapshot and send to me, or adjust 'red' and 'green' thresholds until enough pixels are highlighted on both plates with as few outliers.\n"
            "3. Save the thresholds by changing r_g, r_b, r_n, and so forth, in src/parameters.h.\n"
        );
        End();
    }
}
