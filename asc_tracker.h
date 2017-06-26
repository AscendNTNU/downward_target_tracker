#include "so_math.h"
#include "asc_detector.h"
const int detection_window_count = 60;
const int past_velocity_count = 5*60; // 5 seconds * 60 fps = 300 frames
const int velocity_averaging_window = 20; // Interval of detections used to compute velocity
                                          // => need atleast this many detections before computing velocity
const float target_speed = 0.33f;

struct detection_t
{
    float t;           // Timestamp when detection was made
    float u,v;         // Image coordinates
    float u1,v1,u2,v2; // Bounding box in image
    float x,y;         // World coordinates
};

struct target_t
{
    detection_t last_seen;
    detection_t window[detection_window_count]; // last n detections (latest at [0], oldest at [n-1])
    int num_window;
    int unique_id;

    float velocity_x;
    float velocity_y;

    int num_past_velocity;
    float past_velocity_x[past_velocity_count];
    float past_velocity_y[past_velocity_count];
    float past_velocity_t[past_velocity_count];

    float detection_rate;
    bool observed_180;
    float last_180_time;
};

struct tracks_t
{
    int num_targets;
    target_t *targets;

    int num_detections;
    detection_t *detections;

    float time_until_180;

    int *points;
    int num_points;
    cc_groups groups;
};

struct track_targets_opt_t
{
    // 'red' classification thresholds
    float r_g; // How much stronger red must be over green
    float r_b; // How much stronger red must be over blue
    float r_n; // How bright the average (R+G+B)/3 must be

    // 'green' classification thresholds
    float g_r; // How much stronger green must be over red
    float g_b; // How much stronger green must be over blue
    float g_n; // How bright the average (R+G+B)/3 must be

    // camera calibration
    float f;  // camera focal length
    float u0; // camera center in x
    float v0; // camera center in y

    // input image
    unsigned char *I;  // image of densely packed 24-byte (r,g,b) pixels
    int Ix;            // image width
    int Iy;            // image height

    // pose of camera relative grid
    mat3 rot; // R_c^g: {c}amera coordinate frame relative {g}rid
    vec3 pos; // camera position relative some origin

    // monotonically increasing timer (in seconds) for when image was taken
    float timestamp;
};

tracks_t track_targets(track_targets_opt_t opt);

//
// Implementation
//

vec3 camera_inverse_project(float f, float u0, float v0, vec2 uv)
//  f   (input): Equidistant fisheye camera model parameter (r = f x theta)
// u0   (input): Center of fisheye projection in x measured from left of image
// v0   (input): Center of fisheye projection in y measured from top of image
// uv   (input): Pixel coordinate measured from top-left of image (DirectX convention)
//       return: Camera-space ray from camera origin through pixel (OpenGL convention)
{
    vec3 dir;

    float u = uv.x;
    float v = uv.y;
    float du = u-u0;
    float dv = v0-v;
    float r = sqrtf(du*du+dv*dv);
    if (r > 1.0f)
    {
        float t = r / f;
        float s = sinf(t);
        float c = cosf(t);
        dir.x = s*du/r;
        dir.y = s*dv/r;
        dir.z = -c;
    }
    else
    {
        dir.x = 0.0f;
        dir.y = 0.0f;
        dir.z = -1.0f;
    }

    return dir;
}

float metric_distance(float u1, float v1, float u2, float v2,
                      float f, float u0, float v0, mat3 rot, float h)
{
    vec2 uv1 = { u1, v1 };
    vec2 uv2 = { u2, v2 };
    vec3 dir1 = rot*camera_inverse_project(f,u0,v0, uv1);
    vec3 dir2 = rot*camera_inverse_project(f,u0,v0, uv2);
    vec2 xy1;
    vec2 xy2;
    if (m_intersect_xy_plane(dir1, h, &xy1) &&
        m_intersect_xy_plane(dir2, h, &xy2))
    {
        return m_length(xy1-xy2);
    }
    else
    {
        return 1000.0f;
    }
}

vec2 fit_direction(detection_t *window, int n)
{
    vec2 v = {0};
    float e_033 = FLT_MAX;
    {
        float dx_sum = 0.0f;
        float dy_sum = 0.0f;
        float dt_sum = 0.0f;
        for (int i = 0; i < n; i++)
        {
            float dx = window[i].x - window[n-1].x;
            float dy = window[i].y - window[n-1].y;
            float dt = window[i].t - window[n-1].t;
            dx_sum += dx*dt;
            dy_sum += dy*dt;
            dt_sum += dt*dt;
        }
        if (dt_sum > 0.0f)
            v = m_vec2(dx_sum/dt_sum, dy_sum/dt_sum);
        else
            v = m_vec2(0,0);

        float ds = m_length(v);
        if (ds > 0.0f)
        {
            v *= target_speed/ds;
            e_033 = 0.0f;
            for (int i = 0; i < n; i++)
            {
                float dt = window[i].t - window[n-1].t;
                float dx = window[i].x - (window[n-1].x + v.x*dt);
                float dy = window[i].y - (window[n-1].y + v.y*dt);
                e_033 += dx*dx + dy*dy;
            }
            e_033 /= n;
        }
        else
        {
            e_033 = FLT_MAX;
        }
    }

    float e_zero = FLT_MAX;
    {
        float cx = 0.0f;
        float cy = 0.0f;
        for (int i = 0; i < n; i++)
        {
            cx += window[i].x;
            cy += window[i].y;
        }
        cx /= n;
        cy /= n;
        e_zero = 0.0f;
        for (int i = 0; i < n; i++)
        {
            float dx = window[i].x-cx;
            float dy = window[i].y-cy;
            e_zero += dx*dx + dy*dy;
        }
        e_zero /= n;
    }

    if (e_zero < e_033)
        return m_vec2(0,0);
    else
        return v;
}

bool detect_180_turn(float *vx, float *vy, float *vt, int n, float *last_turn_time)
{
    const float reverse_duration = 1.8f; // arduino source code says 2.15 seconds
                                         // but to be on the safe side
    for (int i = 0; i < n; i++)
    {
        float vx1 = vx[i];
        float vy1 = vy[i];
        float vt1 = vt[i];
        for (int j = i+1; j < n; j++)
        {
            float dot = vx[i]*vx[j] + vy[i]*vy[j];
            // if v_i and v_j are opposite, then
            //   (v_i dot v_j) = |v_i|*|v_j|*-1 = - .33^2 = -0.1089
            // we'll tolerate a +-30 degree noise
            //   cos(150) = cos(210) = -0.87
            //   -0.87 * .33^2 = -0.0947
            float tolerance = -0.87f*target_speed*target_speed;
            bool is_reverse = dot < tolerance;
            if (is_reverse && vt[i] - vt[j] > reverse_duration)
            {
                *last_turn_time = vt[j];
                return true;
            }
        }
    }
    return false;
}

#define rshift(X, COUNT) { for (int LOOPVAR = (COUNT)-1; LOOPVAR > 0; LOOPVAR--) { (X)[LOOPVAR] = (X)[LOOPVAR-1]; } }

void update_target_with_detection(target_t *target, detection_t detection)
{
    target->last_seen = detection;

    // Update window of detections with latest detection
    rshift(target->window, detection_window_count);
    target->window[0] = detection;
    if (target->num_window < detection_window_count)
        target->num_window++;

    // Update velocity
    detection_t *window = target->window;
    vec2 v;
    if (target->num_window >= velocity_averaging_window)
        v = fit_direction(window, velocity_averaging_window);
    else
        v = m_vec2(0,0);
    target->velocity_x = v.x;
    target->velocity_y = v.y;

    rshift(target->past_velocity_x, past_velocity_count);
    rshift(target->past_velocity_y, past_velocity_count);
    rshift(target->past_velocity_t, past_velocity_count);
    target->past_velocity_x[0] = v.x;
    target->past_velocity_y[0] = v.y;
    target->past_velocity_t[0] = detection.t;
    if (target->num_past_velocity < past_velocity_count)
        target->num_past_velocity++;

    float last_180_time;
    if (detect_180_turn(target->past_velocity_x, target->past_velocity_y, target->past_velocity_t, target->num_past_velocity, &last_180_time))
    {
        target->observed_180 = true;
        target->last_180_time = last_180_time;
    }
}

tracks_t track_targets(track_targets_opt_t opt)
{
    const int max_targets = 1024;
    const float plate_z = 0.1f;               // Height of top-plate above ground
    const float merge_threshold = 0.3f;       // Maximum distance in meters for two detections to be considered the same
    const float removal_time = 10.5f;         // Required elapsed time without reobservation before a track is removed
    const int minimum_count = 50;             // Minimum number of pixels inside connected component to be accepted
    const float detection_rate_period = 0.2f; // Time interval used to compute detection rate (hits per second)
    const float frames_per_second = 60.0f;    // Framerate used to normalize detection rate (corresponds to max hits per second)

    // requirements for a color detection to be valid
    const float min_aspect_ratio = 0.2f;      // A detection must be sufficiently square (aspect ~ 1)
    const float max_aspect_ratio = 4.0f;      //
    const float min_fill_percentage = 0.5f;   // Percentage of filled pixels within 2D bounding box

    static target_t targets[max_targets];
    static int num_targets = 0;
    static int next_id = 0;

    float f = opt.f;
    float u0 = opt.u0;
    float v0 = opt.v0;
    mat3 rot = opt.rot;
    vec3 pos = opt.pos;
    float timestamp = opt.timestamp;
    float delta_z = pos.z - plate_z; // Height of camera above target plate plane

    // Detect targets in input image
    static detection_t detections[detector_max_groups];
    int num_detections = 0;
    int *color_points;
    int color_num_points;
    cc_groups color_groups;
    {
        unsigned char *I = opt.I;
        int Ix = opt.Ix;
        int Iy = opt.Iy;

        // Find connected components belonging of top plate sections
        int *points;
        int num_points;
        cc_groups groups;
        {
            cc_options cc_opt = {0};
            cc_opt.r_g = opt.r_g;
            cc_opt.r_b = opt.r_b;
            cc_opt.r_n = opt.r_n;
            cc_opt.g_r = opt.g_r;
            cc_opt.g_b = opt.g_b;
            cc_opt.g_n = opt.g_n;
            groups = cc_find_top_plates(I, Ix, Iy, cc_opt, &points, &num_points);
        }

        // I merge adjacent groups based on ground plane distance
        // This array is used for book-keeping
        static bool merged[detector_max_groups];
        for (int i = 0; i < groups.count; i++)
            merged[i] = false;

        // Merge detections
        for (int i = 0; i < groups.count; i++)
        {
            if (merged[i]) continue;

            for (int j = i+1; j < groups.count; j++)
            {
                if (merged[j]) continue;

                float xi = groups.group_x[i];
                float yi = groups.group_y[i];
                float xj = groups.group_x[j];
                float yj = groups.group_y[j];

                // Check how close the group centroids are in world space
                // and merge only if they are sufficiently close
                float d = metric_distance(xi,yi, xj,yj, f,u0,v0,rot,delta_z);
                if (d > merge_threshold)
                    continue;

                // Compute the new centroid by a weighted average
                int ni = groups.group_n[i];
                int nj = groups.group_n[j];
                float x = (xi*ni + xj*nj)/(float)(ni+nj);
                float y = (yi*ni + yj*nj)/(float)(ni+nj);

                float min_xi = groups.group_min_x[i];
                float min_xj = groups.group_min_x[j];
                float min_yi = groups.group_min_y[i];
                float min_yj = groups.group_min_y[j];

                float max_xi = groups.group_max_x[i];
                float max_xj = groups.group_max_x[j];
                float max_yi = groups.group_max_y[i];
                float max_yj = groups.group_max_y[j];

                // Merge parameters of group j with group i
                groups.group_x[i] = x;
                groups.group_y[i] = y;
                groups.group_n[i] = ni+nj;
                groups.group_min_x[i] = (min_xj < min_xi) ? min_xj : min_xi;
                groups.group_min_y[i] = (min_yj < min_yi) ? min_yj : min_yi;
                groups.group_max_x[i] = (max_xj > max_xi) ? max_xj : max_xi;
                groups.group_max_y[i] = (max_yj > max_yi) ? max_yj : max_yi;

                // Relabel points of group j to group i
                for (int point_index = 0; point_index < num_points; point_index++)
                {
                    int p = points[point_index];
                    if (groups.label[p] == j)
                        groups.label[p] = i;
                }

                merged[j] = true;
            }
        }

        // Compute eigenvalues (used to check aspect ratio)
        static float eigx[detector_max_groups];
        static float eigy[detector_max_groups];
        {
            static float xx[detector_max_groups];
            static float xy[detector_max_groups];
            static float yy[detector_max_groups];
            for (int i = 0; i < groups.count; i++)
            {
                xx[i] = 0.0f;
                xy[i] = 0.0f;
                yy[i] = 0.0f;
                eigx[i] = 0.0f;
                eigy[i] = 0.0f;
            }

            for (int i = 0; i < num_points; i++)
            {
                int p = points[i];
                int x = p % Ix;
                int y = p / Ix;
                int l = groups.label[p];
                float dx = x-groups.group_x[l];
                float dy = y-groups.group_y[l];

                xx[l] += dx*dx;
                xy[l] += dx*dy;
                yy[l] += dy*dy;
            }

            for (int i = 0; i < groups.count; i++)
            {
                float a = xx[i];
                float b = xy[i];
                float c = yy[i];
                float T = a+c;
                float D = a*c-b*b;
                eigx[i] = T/2.0f + sqrtf(T*T/4.0f - D);
                eigy[i] = T/2.0f - sqrtf(T*T/4.0f - D);
            }
        }

        // A valid detection is a connected component that satisfies all of
        //   i) Component has sufficiently many pixels
        //  ii) Its aspect ratio is sufficiently square
        // iii) A sufficiently large portion of bounding box is filled
        for (int i = 0; i < groups.count; i++)
        {
            bool count_ok = groups.group_n[i] >= minimum_count;

            float std1 = sqrtf(eigx[i]);
            float std2 = sqrtf(eigy[i]);
            float aspect = std1/std2;
            bool aspect_ok = aspect >= min_aspect_ratio && aspect <= max_aspect_ratio;

            float min_x = groups.group_min_x[i];
            float min_y = groups.group_min_y[i];
            float max_x = groups.group_max_x[i];
            float max_y = groups.group_max_y[i];
            float area = (1.0f+max_x-min_x)*(1.0f+max_y-min_y);
            float filled = groups.group_n[i] / area;
            bool area_ok = filled > min_fill_percentage;

            if (count_ok && aspect_ok && area_ok && !merged[i])
            {
                detection_t d = {0};
                d.t = timestamp;
                d.u = groups.group_x[i];
                d.v = groups.group_y[i];
                d.u1 = min_x;
                d.u2 = max_x;
                d.v1 = min_y;
                d.v2 = max_y;
                detections[num_detections++] = d;
            }
        }

        color_points = points;
        color_num_points = num_points;
        color_groups = groups;
    }

    // Compute world-space coordinates for each detection using current GPS estimate
    for (int i = 0; i < num_detections; i++)
    {
        vec2 uv = { detections[i].u, detections[i].v };
        vec3 dir = rot*camera_inverse_project(f,u0,v0, uv);
        vec2 xy;
        if (m_intersect_xy_plane(dir, delta_z, &xy))
        {
            detections[i].x = xy.x + pos.x;
            detections[i].y = xy.y + pos.y;
        }
        else
        {
            detections[i].x = -1000.0f;
            detections[i].y = -1000.0f;
        }
    }

    static bool found_match[detector_max_groups];
    for (int i = 0; i < num_detections; i++)
        found_match[i] = false;

    // Update existing tracks
    for (int i = 0; i < num_targets; i++)
    {
        // Remove targets that we haven't seen for a while
        if (timestamp - targets[i].last_seen.t >= removal_time)
        {
            targets[i] = targets[num_targets-1];
            num_targets--;
            i--;
            continue;
        }

        // Find the closest acceptable detection or continue otherwise
        detection_t detection = {0};
        bool reobserving = false;
        {
            int closest_j = -1;
            float closest_d = 0.0f;
            for (int j = 0; j < num_detections; j++)
            {
                float ui = targets[i].last_seen.u;
                float vi = targets[i].last_seen.v;
                float uj = detections[j].u;
                float vj = detections[j].v;
                float dx = targets[i].last_seen.x-detections[j].x;
                float dy = targets[i].last_seen.y-detections[j].y;
                float d = sqrtf(dx*dx + dy*dy);
                if (closest_j < 0 || d < closest_d)
                {
                    closest_j = j;
                    closest_d = d;
                }
            }

            if (closest_j >= 0 && closest_d <= merge_threshold)
            {
                reobserving = true;
                found_match[closest_j] = true;
                detection = detections[closest_j];
            }
        }

        // If the detection was sufficiently close we are reobserving the target
        if (reobserving)
        {
            update_target_with_detection(&targets[i], detection);
        }

        // Update detection rate
        float detection_rate = 0.0f;
        {
            int hits = 0;
            for (int k = 0; k < targets[i].num_window; k++)
            {
                if (timestamp - targets[i].window[k].t <= detection_rate_period)
                    hits++;
            }
            detection_rate = (hits/detection_rate_period) / frames_per_second;
        }
        targets[i].detection_rate = detection_rate;
    }

    // Add new tracks
    for (int i = 0; i < num_detections; i++)
    {
        if (!found_match[i])
        {
            if (num_targets < max_targets)
            {
                target_t t = {0};
                t.last_seen = detections[i];
                t.unique_id = next_id++;
                t.num_window = 0;
                t.detection_rate = 0.0f;
                t.velocity_x = 0.0f;
                t.velocity_y = 0.0f;
                t.num_past_velocity = 0;
                targets[num_targets] = t;
                num_targets++;
            }
        }
    }

    tracks_t result = {0};
    result.num_targets = num_targets;
    result.num_detections = num_detections;
    result.targets = targets;
    result.detections = detections;
    result.points = color_points;
    result.num_points = color_num_points;
    result.groups = color_groups;
    return result;
}
