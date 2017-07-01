#if TESTING_WITH_LAPTOP==1
#define camera_project m_project_pinhole
#define camera_inverse_project m_ray_pinhole
#else
#define camera_project m_project_equidistant
#define camera_inverse_project m_ray_equidistant
#endif

void project_line(float f, float u0, float v0, mat3 R, vec3 T, vec3 p1, vec3 p2)
{
    // transform from world frame to camera frame
    vec3 q1 = m_transpose(R)*(p1 - T);
    vec3 q2 = m_transpose(R)*(p2 - T);

    // only draw if entire line is visible
    if (q1.z < 0.0f && q2.z < 0.0f)
    {
        vec2 s1 = camera_project(f,u0,v0, q1);
        vec2 s2 = camera_project(f,u0,v0, q2);
        ImLine(s1.x, s1.y, s2.x, s2.y);
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

    static bool paused = false;

    static int grid_x = 3;
    static int grid_y = 3;
    static float grid_w = 1.0f;

    // COMPUTE CAMERA POSE RELATIVE GRID
    mat3 imu_rot = m_rotz(imu_rz)*m_roty(imu_ry)*m_rotx(imu_rx);
    vec3 imu_pos = m_vec3(imu_tx, imu_ty, imu_tz);
    mat3 cam_imu_rot = m_rotz(cam_imu_rz)*m_roty(cam_imu_ry)*m_rotx(cam_imu_rx);
    vec3 cam_imu_pos = m_vec3(cam_imu_tx, cam_imu_ty, cam_imu_tz);
    mat3 R = imu_rot*cam_imu_rot;
    vec3 T = imu_pos + imu_rot*cam_imu_pos;

    // DRAW LATEST IMAGE (IF NOT PAUSED)
    {
        vdbOrtho(-1.0f, +1.0f, +1.0f, -1.0f);
        if (!paused)
            vdbSetTexture2D(0, I, Ix, Iy, GL_RGB, GL_UNSIGNED_BYTE, GL_LINEAR, GL_LINEAR);
        vdbDrawTexture2D(0);
    }

    // DRAW GRID / CALIBRATION PATTERN BASED PROJECTED INTO IMAGE
    {
        ImBegin();
        vdbOrtho(0, Ix, Iy, 0);
        ImLineWidth(2.0f);
        ImColor4f(1.0f, 1.0f, 0.2f, 1.0f);
        for (int xi = 0; xi <= grid_x; xi++)
        {
            float x = xi*grid_w;
            for (int i = 0; i < 64; i++)
            {
                float y1 = (i+0)*grid_y*grid_w/64.0f;
                float y2 = (i+1)*grid_y*grid_w/64.0f;
                project_line(f,u0,v0, R,T, m_vec3(x,y1,0), m_vec3(x,y2,0));
            }
        }
        for (int yi = 0; yi <= grid_y; yi++)
        {
            float y = yi*grid_w;
            for (int i = 0; i < 64; i++)
            {
                float x1 = (i+0)*grid_x*grid_w/64.0f;
                float x2 = (i+1)*grid_x*grid_w/64.0f;
                project_line(f,u0,v0, R,T, m_vec3(x1,y,0), m_vec3(x2,y,0));
            }
        }
        ImEnd();
    }

    // CALIBRATION GUI
    SetNextWindowSize(ImVec2(400, 300), ImGuiSetCond_Appearing);
    Begin("Calibration");
    {
        Checkbox("Freeze image", &paused);
        if (CollapsingHeader("Readme (intrinsics)"))
        {
            TextWrapped("Intrinsics define the fisheye parameters. "
                        "They depend on the chosen camera resolution."
                        "They do not depend on the chosen downscaling (CAMERA_LEVELS).");
            TextWrapped("For 800x600: Use f=434, u0=400, v0=300");
            TextWrapped("For 1280x720: Use f=494, u0=649, v0=335");
            TextWrapped("For some other resolution you need to calibrate it yourself, "
                        "or send me a snapshot.");
            TextWrapped("To calibrate:\n"
                        "1. Point the camera at a checkerboard\n"
                        "2. Adjust the \"Calibration pattern\" parameters below to fit\n"
                        "3. Keep the camera at a known rotation and translation "
                        "(f.ex. no rotation and 10 cm away)\n"
                        "4. Adjust f, u0, v0 until yellow lines coincide with checkerboard");
            Separator();
        }
        if (CollapsingHeader("Readme (extrinsics)"))
        {
            TextWrapped("Extrinsics define how camera is mounted relative IMU.");
            TextWrapped("If the only difference between them is a rotation about z, you only need to adjust cam_imu_rz.");
            TextWrapped("You can then verify that this is correct by:\n"
                        "1. Look at a grid pattern\n"
                        "2. Specify how the drone is rotated and translated in the grid\n"
                        "3. Tilt the drone in either x or y axis\n"
                        "4. and verify that visualized grid rotates as expected.");
            Separator();
        }
        if (CollapsingHeader("Calibration pattern"))
        {
            grid_w *= 100.0f; DragFloat("Cell width (cm)", &grid_w); grid_w /= 100.0f;
            DragInt("Cell count x", &grid_x);
            DragInt("Cell count y", &grid_y);
        }
        if (CollapsingHeader("Drone origin relative grid origin in grid coordinates"))
        {
            imu_tx *= 1000.0f; DragFloat("tx (mm)##imu", &imu_tx); imu_tx /= 1000.0f; SameLine(); Checkbox("##imu_tx", &use_mavros_imu_tx);
            imu_ty *= 1000.0f; DragFloat("ty (mm)##imu", &imu_ty); imu_ty /= 1000.0f; SameLine(); Checkbox("##imu_ty", &use_mavros_imu_ty);
            imu_tz *= 1000.0f; DragFloat("tz (mm)##imu", &imu_tz); imu_tz /= 1000.0f; SameLine(); Checkbox("##imu_tz", &use_mavros_imu_tz);
        }
        if (CollapsingHeader("Drone frame relative grid frame"))
        {
            imu_rx *= 180.0f/3.14f; SliderFloat("rx (deg)##imu", &imu_rx, -30.00f, +30.00f); imu_rx *= 3.14f/180.0f; SameLine(); Checkbox("##imu_rx", &use_mavros_imu_rx);
            imu_ry *= 180.0f/3.14f; SliderFloat("ry (deg)##imu", &imu_ry, -30.00f, +30.00f); imu_ry *= 3.14f/180.0f; SameLine(); Checkbox("##imu_ry", &use_mavros_imu_ry);
            imu_rz *= 180.0f/3.14f; SliderFloat("rz (deg)##imu", &imu_rz, -180.0f, +180.0f); imu_rz *= 3.14f/180.0f; SameLine(); Checkbox("##imu_rz", &use_mavros_imu_rz);
        }
        TextWrapped("Untick the checkbox to manually control a value, or otherwise let it be the set to that used in the tracker (i.e. mavros pose).");
    }
    End();
}
