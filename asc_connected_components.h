#ifndef cc_assert
#include <assert.h>
#define cc_assert assert
#endif

#define CC_MAX_WIDTH (1920)
#define CC_MAX_HEIGHT (1080)
#define CC_MAX_POINTS (CC_MAX_WIDTH*CC_MAX_HEIGHT)

struct cc_groups
{
    int   *label;
    float *group_x;
    float *group_y;
    float *group_min_x;
    float *group_min_y;
    float *group_max_x;
    float *group_max_y;
    int   *group_n;
    int    count;
};

cc_groups cc_connected_components(
    unsigned char *binary,
    int width,
    int height,
    int *points,
    int num_points)
{
    cc_assert(width <= CC_MAX_WIDTH);
    cc_assert(height <= CC_MAX_HEIGHT);

    static int   label[CC_MAX_POINTS];
    static float group_x[CC_MAX_POINTS];
    static float group_y[CC_MAX_POINTS];
    static float group_min_x[CC_MAX_POINTS];
    static float group_min_y[CC_MAX_POINTS];
    static float group_max_x[CC_MAX_POINTS];
    static float group_max_y[CC_MAX_POINTS];
    static int   group_n[CC_MAX_POINTS];
    for (int i = 0; i < CC_MAX_POINTS; i++)
    {
        label[i] = -1;
    }

    cc_groups g = {0};
    g.group_x = group_x;
    g.group_y = group_y;
    g.group_min_x = group_min_x;
    g.group_min_y = group_min_y;
    g.group_max_x = group_max_x;
    g.group_max_y = group_max_y;
    g.group_n = group_n;
    g.label = label;
    g.count = 0;

    for (int i_p = 0; i_p < num_points; i_p++)
    {
        int root = points[i_p];
        if (label[root] == -1)
        {
            float sum_x = 0.0f;
            float sum_y = 0.0f;
            float min_x = (float)width;
            float max_x = 0.0f;
            float min_y = (float)height;
            float max_y = 0.0f;
            int sum_n = 0;

            static int queue[CC_MAX_POINTS];
            queue[0] = root;
            int queue_count = 1;
            while (queue_count > 0)
            {
                // extract from queue and grow component
                int p = queue[--queue_count];
                label[p] = g.count;
                float x = (float)(p % width);
                float y = (float)(p / width);
                sum_x += x;
                sum_y += y;
                min_x = (x < min_x) ? x : min_x;
                min_y = (y < min_y) ? y : min_y;
                max_x = (x > max_x) ? x : max_x;
                max_y = (y > max_y) ? y : max_y;
                sum_n += 1;

                // add neighbors
                int neighbors[] = {
                    p+width,
                    p-width,
                    p+1,
                    p-1,
                    p+1+width,
                    p-1+width,
                    p+1-width,
                    p-1-width
                };
                for (int i = 0; i < 8; i++)
                {
                    int q = neighbors[i];
                    if (binary[q] > 0 && g.label[q] == -1)
                    {
                        queue[queue_count++] = q;
                        cc_assert(queue_count <= CC_MAX_POINTS);
                    }
                }
            }

            g.group_x[g.count] = sum_x / (float)sum_n;
            g.group_y[g.count] = sum_y / (float)sum_n;
            g.group_min_x[g.count] = min_x;
            g.group_min_y[g.count] = min_y;
            g.group_max_x[g.count] = max_x;
            g.group_max_y[g.count] = max_y;
            g.group_n[g.count] = sum_n;
            g.count++;
        }
    }
    return g;
}

struct cc_options
{
    float r_g;
    float r_b;
    float r_n;
    float g_r;
    float g_b;
    float g_n;
};

cc_groups cc_find_top_plates(unsigned char *rgb, int width, int height, cc_options opt,
                             int **opt_points = 0, int *opt_num_points = 0)
{
    static int points[CC_MAX_POINTS];
    static unsigned char binary[CC_MAX_POINTS];

    int num_points = 0;
    for (int y = 1; y < height-1; y++)
    for (int x = 1; x < width-1; x++)
    {
        float r = (float)rgb[(x+y*width)*3+0];
        float g = (float)rgb[(x+y*width)*3+1];
        float b = (float)rgb[(x+y*width)*3+2];
        float norm = r + g + b;
        if (norm > 0.0f)
        {
            r /= norm;
            g /= norm;
            b /= norm;

            bool is_red = r > opt.r_g*g && r > opt.r_b*b && norm > opt.r_n*3;
            bool is_grn = g > opt.g_r*r && g > opt.g_b*b && norm > opt.g_n*3;
            if (is_red || is_grn)
            {
                points[num_points++] = y*width+x;
                binary[y*width+x] = 1;
            }
            else
            {
                binary[y*width+x] = 0;
            }
        }
    }

    cc_groups groups = cc_connected_components(binary, width, height, points, num_points);

    if (opt_points && opt_num_points)
    {
        *opt_points = points;
        *opt_num_points = num_points;
    }

    return groups;
}
