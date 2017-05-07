#include "so_math.h"
#include "asc_connected_components.h"

struct detection_t
{
    float t;   // Timestamp
    float u,v; // Image coordinates
    float u1,v1,u2,v2; // Bounding box in image
    float x_gps,y_gps; // World coordinates
    bool has_gps;
};

struct target_t
{
    int confidence;
    detection_t last_seen;
    float u_hat;
    float v_hat;
    float du_hat;
    float dv_hat;
    int unique_id;

    float x_hat;
    float y_hat;
    float dx_hat;
    float dy_hat;
    float p_turning;
    float p_moving;
};

struct tracks_t
{
    int num_targets;
    target_t *targets;

    int num_detections;
    detection_t *detections;

    // color segmentation output
    int *points;
    int num_points;
    cc_groups groups;
};

struct track_targets_opt_t
{
    // Height of top-plate above ground
    float platez; // 0.1

    // Maximum distance in meters for two detections to be considered the same
    float merge_threshold; // 0.3

    // Confidence is increased or decreased by one each frame
    int confidence_limit; // 20
    int initial_confidence; // 5
    int accept_confidence; // 10
    int removal_confidence; // 0

    // Required elapsed time without reobservation before a track is removed
    // (in same unit as timestamp)
    float removal_time; // 2.0f

    // Minimum number of pixels inside connected component to be accepted
    int minimum_count; // 50

    // Color segmentation thresholds
    float r_g; // 3.0f
    float r_b; // 3.0f
    float r_n; // 10.0f/3.0f
    float g_r; // 1.6f
    float g_b; // 1.5f
    float g_n; // 10.0f/3.0f

    float f;  // camera focal length
    float u0; // camera center in x
    float v0; // camera center in y

    unsigned char *I;  // image of densely packed 24-byte (r,g,b) pixels
    int Ix;            // image width
    int Iy;            // image height

    mat3 rot; // R_c^g: {c}amera coordinate frame relative {g}rid
    vec3 pos; // camera position relative some origin
    bool gps; // whether rot/pos are valid or not

    float timestamp; // monotonically increasing timer (in seconds) for when image was taken
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

target_t filter_target_image_space(target_t prev, detection_t seen)
{
    //
    // Predict image-space position dt time units ahead
    //
    float dt = seen.t - prev.last_seen.t;
    float u_pre = prev.u_hat + prev.du_hat*dt;
    float v_pre = prev.v_hat + prev.dv_hat*dt;

    //
    // Lowpass-filter image-space position and velocity
    //
    float kp = 0.7f;                                         // filter stickiness for position
    float kd = 0.9f;                                         // filter stickiness for velocity
    float u_hat = kp*u_pre + (1.0f-kp)*seen.u;               // lowpass-filtered position (x)
    float v_hat = kp*v_pre + (1.0f-kp)*seen.v;               // lowpass-filtered position (y)
    float du_obs = (u_hat - prev.u_hat)/dt;                  // 'observed' velocity (x)
    float dv_obs = (v_hat - prev.v_hat)/dt;                  // 'observed' velocity (y)
    float du_hat = 0.8f*(kd*prev.du_hat + (1.0f-kd)*du_obs); // lowpass-filtered velocity (x)
    float dv_hat = 0.8f*(kd*prev.dv_hat + (1.0f-kd)*dv_obs); // lowpass-filtered velocity (y)

    //
    // Predict world-space motion dt time units ahead
    //
    float x_pre = prev.x_hat + prev.dx_hat*dt;               // predicted position (x)
    float y_pre = prev.y_hat + prev.dy_hat*dt;               // predicted position (y)
    float dx_pre = prev.dx_hat;                              // predicted velocity (x)
    float dy_pre = prev.dy_hat;                              // predicted velocity (y)

    if (seen.has_gps)
    {
        float kp = 0.7f;
        float kd = 0.8f;
        float x_hat = kp*x_pre + (1.0f-kp)*seen.x_gps;
        float y_hat = kp*y_pre + (1.0f-kp)*seen.y_gps;
        float dx_obs = (x_hat - prev.x_hat)/dt;
        float dy_obs = (y_hat - prev.y_hat)/dt;
        float dx_hat = kd*dx_pre + (1.0f-kd)*dx_obs;
        float dy_hat = kd*dy_pre + (1.0f-kd)*dy_obs;

        // estimate mode distribution
        float kt = 0.2f;
        float magnitude = sqrtf(dx_obs*dx_obs + dy_obs*dy_obs);
        if (magnitude < 0.15f)
        {
            prev.p_turning += (1.0f - prev.p_turning)*(dt/kt);
            prev.p_moving  += (0.0f - prev.p_moving)*(dt/kt);
        }
        else if (magnitude > 0.15f && magnitude < 0.4f)
        {
            prev.p_turning += (0.0f - prev.p_turning)*(dt/kt);
            prev.p_moving  += (1.0f - prev.p_moving)*(dt/kt);
        }
        else
        {
            prev.p_turning += (0.0f - prev.p_turning)*(dt/kt);
            prev.p_moving  += (0.0f - prev.p_moving)*(dt/kt);
        }
        prev.x_hat = seen.x_gps;
        prev.y_hat = seen.y_gps;
        prev.dx_hat = dx_hat;
        prev.dy_hat = dy_hat;
    }
    else
    {
        prev.x_hat = x_pre;
        prev.y_hat = y_pre;
        prev.dx_hat = dx_pre;
        prev.dy_hat = dy_pre;
    }

    // todo: debug: removeme
    #if 0
    if (seen.has_gps)
    {
        const int window_cap = 60;
        static int window_len = 0;
        static float x_gps_history[window_cap];
        static float y_gps_history[window_cap];
        static float t_gps_history[window_cap];
        if (window_len < window_cap)
        {
            x_gps_history[window_len] = seen.x_gps;
            y_gps_history[window_len] = seen.y_gps;
            t_gps_history[window_len] = seen.t;
            window_len++;
            prev.dx_hat = 0.0f;
            prev.dy_hat = 0.0f;
        }

        if (window_len == window_cap)
        {
            prev.dx_hat = (seen.x_gps - x_gps_history[0]) / (seen.t - t_gps_history[0]);
            prev.dy_hat = (seen.y_gps - y_gps_history[0]) / (seen.t - t_gps_history[0]);

            for (int i = 0; i < window_cap-1; i++)
            {
                x_gps_history[i] = x_gps_history[i+1];
                y_gps_history[i] = y_gps_history[i+1];
                t_gps_history[i] = t_gps_history[i+1];
            }
            window_len--;
        }
    }
    #endif

    prev.last_seen = seen;
    prev.u_hat = u_hat;
    prev.v_hat = v_hat;
    prev.du_hat = du_hat;
    prev.dv_hat = dv_hat;
    return prev;
}

tracks_t track_targets(track_targets_opt_t opt)
{
    const int MAX_TARGETS = 1024;
    static target_t targets[MAX_TARGETS];
    static int num_targets = 0;
    static int next_id = 0;

    float f = opt.f;
    float u0 = opt.u0;
    float v0 = opt.v0;

    unsigned char *I = opt.I;
    int Ix = opt.Ix;
    int Iy = opt.Iy;

    mat3 rot = opt.rot;
    vec3 pos = opt.pos;
    bool has_gps = opt.gps;

    float timestamp = opt.timestamp;

    // Height of top-plate above ground
    float platez = opt.platez;

    // Height of camera above target plate plane
    float deltah = pos.z-platez;

    // Maximum distance in meters for two detections to be considered the same
    float merge_threshold = opt.merge_threshold;

    // Confidence is increased or decreased by one each frame
    int confidence_limit = opt.confidence_limit;
    int initial_confidence = opt.initial_confidence;
    int accept_confidence = opt.accept_confidence;
    int removal_confidence = opt.removal_confidence;

    // Required elapsed time without reobservation before a track is removed
    // (in same unit as timestamp)
    float removal_time = opt.removal_time;

    // Minimum number of pixels inside connected component to be accepted
    int minimum_count = opt.minimum_count;

    static detection_t detections[CC_MAX_POINTS];
    int num_detections = 0;

    // Detect targets in input image
    int *color_points;
    int color_num_points;
    cc_groups color_groups;
    {
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
        static bool merged[CC_MAX_POINTS];
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
                float d = metric_distance(xi,yi, xj,yj, f,u0,v0,rot,deltah);
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
        static float eigx[CC_MAX_POINTS];
        static float eigy[CC_MAX_POINTS];
        {
            static float xx[CC_MAX_POINTS];
            static float xy[CC_MAX_POINTS];
            static float yy[CC_MAX_POINTS];
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
            bool aspect_ok = aspect >= 0.2f && aspect <= 4.0f;

            float min_x = groups.group_min_x[i];
            float min_y = groups.group_min_y[i];
            float max_x = groups.group_max_x[i];
            float max_y = groups.group_max_y[i];
            float area = (1.0f+max_x-min_x)*(1.0f+max_y-min_y);
            float filled = groups.group_n[i] / area;
            bool area_ok = filled > 0.5f;

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
        if (m_intersect_xy_plane(dir, deltah, &xy))
        {
            detections[i].x_gps = xy.x + pos.x;
            detections[i].y_gps = xy.y + pos.y;
            detections[i].has_gps = true;
        }
        else
        {
            detections[i].has_gps = false;
        }
    }

    static bool found_match[CC_MAX_POINTS];
    for (int i = 0; i < num_detections; i++)
        found_match[i] = false;

    // Update existing tracks
    for (int i = 0; i < num_targets; i++)
    {
        // First, find the closest detection
        int closest_j = -1;
        float closest_d = 0.0f;
        for (int j = 0; j < num_detections; j++)
        {
            float ui = targets[i].u_hat;
            float vi = targets[i].v_hat;
            float uj = detections[j].u;
            float vj = detections[j].v;
            float d = metric_distance(ui,vi, uj,vj, f,u0,v0,rot,deltah);
            if (closest_j < 0 || d < closest_d)
            {
                closest_j = j;
                closest_d = d;
            }
        }

        // If the detection is sufficiently close we are reobserving the same target
        if (closest_j >= 0 && closest_d < merge_threshold)
        {
            // @ todo: merge neighboring detections _during_ filtering
            found_match[closest_j] = true;
            targets[i] = filter_target_image_space(targets[i], detections[closest_j]);

            if (targets[i].confidence < confidence_limit)
                targets[i].confidence++;
        }
        else
        {
            if (targets[i].confidence > 0)
                targets[i].confidence--;
        }

        bool should_remove = (targets[i].confidence <= removal_confidence) ||
                             (timestamp - targets[i].last_seen.t >= removal_time);
        if (should_remove)
        {
            targets[i] = targets[num_targets-1];
            num_targets--;
            i--;
        }
    }

    // Add new tracks
    for (int i = 0; i < num_detections; i++)
    {
        if (!found_match[i])
        {
            if (num_targets < MAX_TARGETS)
            {
                target_t t = {0};
                t.confidence = initial_confidence;
                t.last_seen = detections[i];
                t.u_hat = detections[i].u;
                t.v_hat = detections[i].v;
                t.du_hat = 0.0f;
                t.dv_hat = 0.0f;

                t.x_hat = detections[i].x_gps;
                t.y_hat = detections[i].y_gps;
                t.dx_hat = 0.0f;
                t.dy_hat = 0.0f;
                t.p_moving = 0.8f;
                t.p_turning = 0.2f;

                t.unique_id = next_id++;
                targets[num_targets] = t;
                num_targets++;
            }
        }
    }

    #if 0
    static int selected_id = -1;
    if (vdb_begin())
    {
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
