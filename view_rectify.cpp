#include <math.h>

vec2 camera_project(float f, float u0, float v0, vec3 p)
//  f  (input): Equidistant fisheye camera model parameter (r = f x theta)
// u0  (input): Center of fisheye projection in x measured from left of image
// v0  (input): Center of fisheye projection in y measured from top of image
// p   (input): Camera-space coordinate (OpenGL convention)
// uv (output): Pixel coordinate measured from top-left of image (DirectX convention)
{
    float l = sqrtf(p.x*p.x+p.y*p.y);
    if (l < 0.001f)
    {
        return m_vec2(u0, v0);
    }
    else
    {
        float t = atanf(-l/p.z);
        float r = f*t;
        return m_vec2(u0 + r*p.x/l, v0 - r*p.y/l);
    }
}

void view_rectify(latest_image_t latest_image, downward_target_tracker::info latest_info)
{
    using namespace ImGui;
    if (!latest_image.I)
        return;

    float f = latest_info.camera_f*latest_image.Ix/latest_info.camera_w;
    float u0 = latest_info.camera_u0*latest_image.Ix/latest_info.camera_w;
    float v0 = latest_info.camera_v0*latest_image.Ix/latest_info.camera_w;
    float cam_imu_rx = latest_info.cam_imu_rx;
    float cam_imu_ry = latest_info.cam_imu_ry;
    float cam_imu_rz = latest_info.cam_imu_rz;
    float cam_imu_tx = latest_info.cam_imu_tx;
    float cam_imu_ty = latest_info.cam_imu_ty;
    float cam_imu_tz = latest_info.cam_imu_tz;

    unsigned char *I = latest_image.I;
    int Ix = latest_image.Ix;
    int Iy = latest_image.Iy;

    static bool use_mavros_imu_rx = true;  static float imu_rx = 0.0f; if (use_mavros_imu_rx) imu_rx = latest_info.imu_rx;
    static bool use_mavros_imu_ry = true;  static float imu_ry = 0.0f; if (use_mavros_imu_ry) imu_ry = latest_info.imu_ry;
    static bool use_mavros_imu_rz = true;  static float imu_rz = 0.0f; if (use_mavros_imu_rz) imu_rz = latest_info.imu_rz;
    static bool use_mavros_imu_tx = false; static float imu_tx = 0.0f; if (use_mavros_imu_tx) imu_tx = latest_info.imu_tx;
    static bool use_mavros_imu_ty = false; static float imu_ty = 0.0f; if (use_mavros_imu_ty) imu_ty = latest_info.imu_ty;
    static bool use_mavros_imu_tz = true;  static float imu_tz = 1.0f; if (use_mavros_imu_tz) imu_tz = latest_info.imu_tz;

    static int grid_x = 3;
    static int grid_y = 3;
    static float grid_w = 1.0f;

    mat3 imu_rot = m_rotz(imu_rz)*m_roty(imu_ry)*m_rotx(imu_rx);
    vec3 imu_pos = m_vec3(imu_tx, imu_ty, imu_tz);
    mat3 cam_imu_rot = m_rotz(cam_imu_rz)*m_roty(cam_imu_ry)*m_rotx(cam_imu_rx);
    vec3 cam_imu_pos = m_vec3(cam_imu_tx, cam_imu_ty, cam_imu_tz);
    mat3 R = imu_rot*cam_imu_rot;
    vec3 T = imu_pos + imu_rot*cam_imu_pos;

    vdbOrtho(-1.0f, +1.0f, +1.0f, -1.0f);
    vdbSetTexture2D(0, I, Ix, Iy, GL_RGB, GL_UNSIGNED_BYTE, GL_LINEAR, GL_LINEAR);
    vdbDrawTexture2D(0);

    vdbOrtho(0.0f, Ix, Iy, 0.0f);
    glLines(2.0f);
    glColor4f(1.0f, 1.0f, 0.2f, 1.0f);

    for (int xi = 0; xi <= grid_x; xi++)
    {
        float x = xi*grid_w;
        for (int i = 0; i < 64; i++)
        {
            float y1 = (i+0)*grid_y*grid_w/64.0f;
            float y2 = (i+1)*grid_y*grid_w/64.0f;
            vec2 s1 = camera_project(f,u0,v0, m_transpose(R)*(m_vec3(x, y1, 0) - T));
            vec2 s2 = camera_project(f,u0,v0, m_transpose(R)*(m_vec3(x, y2, 0) - T));
            glVertex2f(s1.x, s1.y);
            glVertex2f(s2.x, s2.y);
        }
    }

    for (int yi = 0; yi <= grid_y; yi++)
    {
        float y = yi*grid_w;
        for (int i = 0; i < 64; i++)
        {
            float x1 = (i+0)*grid_x*grid_w/64.0f;
            float x2 = (i+1)*grid_x*grid_w/64.0f;
            vec2 s1 = camera_project(f,u0,v0, m_transpose(R)*(m_vec3(x1, y, 0) - T));
            vec2 s2 = camera_project(f,u0,v0, m_transpose(R)*(m_vec3(x2, y, 0) - T));
            glVertex2f(s1.x, s1.y);
            glVertex2f(s2.x, s2.y);
        }
    }
    glEnd();

    Begin("Calibration");
    Text("Calibration pattern:");
    grid_w *= 100.0f; DragFloat("Cell width (cm)", &grid_w); grid_w /= 100.0f;
    DragInt("Cell count x", &grid_x);
    DragInt("Cell count y", &grid_y);
    Text("Drone origin relative grid origin in grid coordinates");
    imu_tx *= 100.0f; DragFloat("tx (cm)##imu", &imu_tx); imu_tx /= 100.0f; SameLine(); Checkbox("##imu_tx", &use_mavros_imu_tx);
    imu_ty *= 100.0f; DragFloat("ty (cm)##imu", &imu_ty); imu_ty /= 100.0f; SameLine(); Checkbox("##imu_ty", &use_mavros_imu_ty);
    imu_tz *= 100.0f; DragFloat("tz (cm)##imu", &imu_tz); imu_tz /= 100.0f; SameLine(); Checkbox("##imu_tz", &use_mavros_imu_tz);
    Text("Drone frame relative grid frame");
    imu_rx *= 180.0f/3.14f; SliderFloat("rx (deg)##imu", &imu_rx, -60.00f, +60.00f); imu_rx *= 3.14f/180.0f; SameLine(); Checkbox("##imu_rx", &use_mavros_imu_rx);
    imu_ry *= 180.0f/3.14f; SliderFloat("ry (deg)##imu", &imu_ry, -60.00f, +60.00f); imu_ry *= 3.14f/180.0f; SameLine(); Checkbox("##imu_ry", &use_mavros_imu_ry);
    imu_rz *= 180.0f/3.14f; SliderFloat("rz (deg)##imu", &imu_rz, -180.0f, +180.0f); imu_rz *= 3.14f/180.0f; SameLine(); Checkbox("##imu_rz", &use_mavros_imu_rz);
    Text("Untick the checkbox to manually control a value");
    End();
}
