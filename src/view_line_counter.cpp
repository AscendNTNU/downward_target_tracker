void view_line_counter(latest_image_t latest_image,
                       downward_target_tracker::info latest_info,
                       ascend_msgs::LineCounter latest_line_counter)
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
    float imu_rx = latest_info.imu_rx;
    float imu_ry = latest_info.imu_ry;
    float imu_rz = latest_line_counter.yaw1;
    float imu_tx = latest_line_counter.x1;
    float imu_ty = latest_line_counter.y1;
    float imu_tz = latest_info.imu_tz;

    // COMPUTE CAMERA POSE RELATIVE GRID
    mat3 imu_rot = m_rotz(imu_rz)*m_roty(imu_ry)*m_rotx(imu_rx);
    vec3 imu_pos = m_vec3(imu_tx, imu_ty, imu_tz);
    mat3 cam_imu_rot = m_rotz(cam_imu_rz)*m_roty(cam_imu_ry)*m_rotx(cam_imu_rx);
    vec3 cam_imu_pos = m_vec3(cam_imu_tx, cam_imu_ty, cam_imu_tz);
    mat3 R = imu_rot*cam_imu_rot;
    vec3 T = imu_pos + imu_rot*cam_imu_pos;

    unsigned char *I = latest_image.I;
    int Ix = latest_image.Ix;
    int Iy = latest_image.Iy;

    // DRAW CAMERA FRAME
    vdbOrtho(-1.0f, +1.0f, +1.0f, -1.0f);
    vdbSetTexture2D(0, I, Ix, Iy, GL_RGB, GL_UNSIGNED_BYTE, GL_LINEAR, GL_LINEAR);
    vdbDrawTexture2D(0);

    // DRAW GRID
    {
        float grid_w = latest_info.tile_width;
        int grid_x = 20;
        int grid_y = 20;
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
}
