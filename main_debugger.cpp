#include "vdb/vdb.h"
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <downward_target_tracker/image.h>
#include <downward_target_tracker/info.h>
#include <downward_target_tracker/tracks.h>
#include <turbojpeg.h>
#include "so_math.h"
#include "view_rectify.cpp"
#include "view_tracks.cpp"
#include "view_color.cpp"

struct latest_image_t
{
    unsigned char *jpg_data;
    unsigned int jpg_size;
    unsigned char *I;
    int Ix;
    int Iy;
};

latest_image_t latest_image = {0};
downward_target_tracker::info latest_info;
downward_target_tracker::tracks latest_tracks;

void callback_info(downward_target_tracker::info msg) { latest_info = msg; }
void callback_tracks(downward_target_tracker::tracks msg) { latest_tracks = msg; }
void callback_image(downward_target_tracker::image msg)
{
    static tjhandle decompressor = tjInitDecompress();
    int subsamp,width,height,error;

    unsigned char *jpg_data = (unsigned char*)&msg.jpg_data[0];
    unsigned int jpg_size = (unsigned int)msg.jpg_data.size();

    error = tjDecompressHeader2(decompressor,
        jpg_data,
        jpg_size,
        &width,
        &height,
        &subsamp);

    if (error)
    {
        printf("%s\n", tjGetErrorStr());
        return;
    }

    if (latest_image.jpg_data)
    {
        free(latest_image.jpg_data);
        latest_image.jpg_data = 0;
    }

    latest_image.jpg_data = (unsigned char*)malloc(jpg_size);
    memcpy(latest_image.jpg_data, jpg_data, jpg_size);
    latest_image.jpg_size = jpg_size;

    if (latest_image.I)
    {
        free(latest_image.I);
        latest_image.I = 0;
    }

    latest_image.I = (unsigned char*)malloc(width*height*3);
    latest_image.Ix = width/4;
    latest_image.Iy = height/4;

    error = tjDecompress2(decompressor,
        jpg_data,
        jpg_size,
        latest_image.I,
        latest_image.Ix,
        0,
        latest_image.Iy,
        TJPF_RGB,
        TJFLAG_FASTDCT);

    if (error)
    {
        printf("%s\n", tjGetErrorStr());
        free(latest_image.I);
        latest_image.I = 0;
        return;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "downward_target_debug");
    ros::NodeHandle node;
    ros::Subscriber sub_image  = node.subscribe("/downward_target_tracker/image", 1, callback_image);
    ros::Subscriber sub_info   = node.subscribe("/downward_target_tracker/info", 1, callback_info);
    ros::Subscriber sub_tracks = node.subscribe("/downward_target_tracker/tracks", 1, callback_tracks);

    int selected_id = -1;

    VDBB("downward_target_debug");
    {
        const int mode_see_tracks = 0;
        const int mode_camera_calibration = 1;
        const int mode_color_calibration = 2;
        static int mode = mode_see_tracks;

        if (latest_image.I && mode == mode_see_tracks)
        {
            unsigned char *I = latest_image.I;
            int Ix = latest_image.Ix;
            int Iy = latest_image.Iy;
            vdbOrtho(-1.0f, +1.0f, +1.0f, -1.0f);
            vdbSetTexture2D(0, I, Ix, Iy, GL_RGB, GL_UNSIGNED_BYTE, GL_NEAREST, GL_NEAREST);
            vdbDrawTexture2D(0);
            view_tracks(latest_info, latest_tracks, selected_id);

            Begin("Select target");
            {
                for (int i = 0; i < latest_tracks.num_targets; i++)
                {
                    Columns(4, "latest_tracks");
                    Separator();
                    Text("ID");        NextColumn();
                    Text("Pos (xy)");  NextColumn();
                    Text("Vel (xy)");  NextColumn();
                    Text("Hitrate");   NextColumn();
                    Separator();
                    for (int i = 0; i < latest_tracks.num_targets; i++)
                    {
                        char label[32];
                        sprintf(label, "%d", latest_tracks.unique_id[i]);
                        if (Selectable(label, selected_id == latest_tracks.unique_id[i], ImGuiSelectableFlags_SpanAllColumns))
                        {
                            if (selected_id == latest_tracks.unique_id[i])
                                selected_id = -1;
                            else
                                selected_id = latest_tracks.unique_id[i];
                        }
                        NextColumn();

                        ImVec4 color = ImVec4(1.0f,1.0f,1.0f,1.0f);
                        if (selected_id == latest_tracks.unique_id[i])
                            color = ImVec4(1.0f, 0.3f, 0.1f, 1.0f);

                        TextColored(color, "%.2f %.2f", latest_tracks.position_x[i], latest_tracks.position_y[i]); NextColumn();
                        TextColored(color, "%.2f %.2f", latest_tracks.velocity_x[i], latest_tracks.velocity_y[i]); NextColumn();
                        TextColored(color, "%.2f",      latest_tracks.detection_rate[i]);                          NextColumn();
                    }
                    Columns(1);
                    Separator();
                }

                bool selected_id_exists = false;
                for (int i = 0; i < latest_tracks.num_targets; i++)
                {
                    if (latest_tracks.unique_id[i] == selected_id)
                        selected_id_exists = true;
                }
                if (!selected_id_exists && selected_id != -1)
                    Text("Target (%d) was lost", selected_id);
            }
            End();
        }

        if (latest_image.I && mode == mode_color_calibration)
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
            view_color(I, Ix, Iy, r_g, r_b, r_n, g_r, g_b, g_n);
        }

        if (latest_image.I && mode == mode_camera_calibration)
        {
            float f = latest_info.camera_f*latest_image.Ix/latest_info.camera_w;
            float u0 = latest_info.camera_u0*latest_image.Ix/latest_info.camera_w;
            float v0 = latest_info.camera_v0*latest_image.Ix/latest_info.camera_w;
            unsigned char *I = latest_image.I;
            int Ix = latest_image.Ix;
            int Iy = latest_image.Iy;
            float cam_imu_rx = latest_info.cam_imu_rx;
            float cam_imu_ry = latest_info.cam_imu_ry;
            float cam_imu_rz = latest_info.cam_imu_rz;
            float cam_imu_tx = latest_info.cam_imu_tx;
            float cam_imu_ty = latest_info.cam_imu_ty;
            float cam_imu_tz = latest_info.cam_imu_tz;
            float imu_rx = latest_info.imu_rx;
            float imu_ry = latest_info.imu_ry;
            float imu_rz = latest_info.imu_rz;
            float imu_tx = latest_info.imu_tx;
            float imu_ty = latest_info.imu_ty;
            float imu_tz = latest_info.imu_tz;

            mat3 imu_rot = m_rotz(imu_rz)*m_roty(imu_ry)*m_rotx(imu_rx);
            vec3 imu_pos = m_vec3(imu_tx, imu_ty, imu_tz);
            mat3 cam_imu_rot = m_rotz(cam_imu_rz)*m_roty(cam_imu_ry)*m_rotx(cam_imu_rx);
            vec3 cam_imu_pos = m_vec3(cam_imu_tx, cam_imu_ty, cam_imu_tz);
            mat3 cam_rot = imu_rot*cam_imu_rot;
            vec3 cam_pos = imu_pos + imu_rot*cam_imu_pos;
            view_rectify(I, Ix, Iy, f, u0, v0, cam_rot, cam_pos);
        }

        Begin("Timing");
        {
            Columns(2, "columns_timing");
            Text("Output rate");  NextColumn(); Text("%.2f Hz", 1.0f/latest_info.dt_cycle);            NextColumn(); Separator();
            Text("Frame rate");   NextColumn(); Text("%.2f Hz", 1.0f/latest_info.dt_frame);            NextColumn(); Separator();
            Text("MJPEG to RGB"); NextColumn(); Text("%.2f ms", 1000.0f*latest_info.dt_jpeg_to_rgb);   NextColumn(); Separator();
            Text("Tracker");      NextColumn(); Text("%.2f ms", 1000.0f*latest_info.dt_track_targets); NextColumn(); Separator();
            Columns(1);

            if (latest_info.dt_cycle > 1.0f/60.0f)
                TextColored(ImVec4(1.0f,0.3f,0.1f,1.0f), "Output rate is less than 60 Hz\nSend Simen a message on Slack.");
        }
        End();

        if (mode == mode_color_calibration || mode == mode_camera_calibration)
        {
            Begin("Parameters");
            {
                static bool locked = true;
                Checkbox("Locked", &locked);
                static float camera_f = 0.0f;
                static float camera_u0 = 0.0f;
                static float camera_v0 = 0.0f;
                static float cam_imu_rx = 0.0f;
                static float cam_imu_ry = 0.0f;
                static float cam_imu_rz = 0.0f;
                static float cam_imu_tx = 0.0f;
                static float cam_imu_ty = 0.0f;
                static float cam_imu_tz = 0.0f;
                static float r_g = 0.0f;
                static float r_b = 0.0f;
                static float r_n = 0.0f;
                static float g_r = 0.0f;
                static float g_b = 0.0f;
                static float g_n = 0.0f;
                if (locked)
                {
                    Text("Camera intrinsics");
                    Separator();
                    camera_f = latest_info.camera_f; Text("camera_f: %f", camera_f);
                    camera_u0 = latest_info.camera_u0; Text("camera_u0: %f", camera_u0);
                    camera_v0 = latest_info.camera_v0; Text("camera_v0: %f", camera_v0);
                    Separator();
                    Text("Camera extrinsics");
                    Separator();
                    cam_imu_rx = latest_info.cam_imu_rx; Text("cam_imu_rx: %f", cam_imu_rx);
                    cam_imu_ry = latest_info.cam_imu_ry; Text("cam_imu_ry: %f", cam_imu_ry);
                    cam_imu_rz = latest_info.cam_imu_rz; Text("cam_imu_rz: %f", cam_imu_rz);
                    cam_imu_tx = latest_info.cam_imu_tx; Text("cam_imu_tx: %f", cam_imu_tx);
                    cam_imu_ty = latest_info.cam_imu_ty; Text("cam_imu_ty: %f", cam_imu_ty);
                    cam_imu_tz = latest_info.cam_imu_tz; Text("cam_imu_tz: %f", cam_imu_tz);
                    Separator();
                    Text("Red thresholds");
                    Separator();
                    r_g = latest_info.r_g; Text("r_g: %f", r_g);
                    r_b = latest_info.r_b; Text("r_b: %f", r_b);
                    r_n = latest_info.r_n; Text("r_n: %f", r_n);
                    Separator();
                    Text("Green thresholds");
                    Separator();
                    g_r = latest_info.g_r; Text("g_r: %f", g_r);
                    g_b = latest_info.g_b; Text("g_b: %f", g_b);
                    g_n = latest_info.g_n; Text("g_n: %f", g_n);
                    Separator();
                }
                else
                {
                    if (CollapsingHeader("Camera intrinsics"))
                    {
                        DragFloat("camera_f", &camera_f);
                        DragFloat("camera_u0", &camera_u0);
                        DragFloat("camera_v0", &camera_v0);
                    }
                    if (CollapsingHeader("Camera extrinsics"))
                    {
                        DragFloat("cam_imu_rx", &cam_imu_rx);
                        DragFloat("cam_imu_ry", &cam_imu_ry);
                        DragFloat("cam_imu_rz", &cam_imu_rz);
                        DragFloat("cam_imu_tx", &cam_imu_tx);
                        DragFloat("cam_imu_ty", &cam_imu_ty);
                        DragFloat("cam_imu_tz", &cam_imu_tz);
                    }
                    if (CollapsingHeader("Red thresholds"))
                    {
                        DragFloat("red over green (r_g)", &r_g);
                        DragFloat("red over blue (r_b)", &r_b);
                        DragFloat("minimum brightness (r_n)", &r_n);
                    }
                    if (CollapsingHeader("Green thresholds"))
                    {
                        DragFloat("green over red (g_r)", &g_r);
                        DragFloat("green over blue (g_b)", &g_b);
                        DragFloat("minimum brightness (g_n)", &g_n);
                    }

                    static ros::Publisher pub_camera_f = node.advertise<std_msgs::Float32>("/downward_target_debug/camera_f", 1);
                    static ros::Publisher pub_camera_u0 = node.advertise<std_msgs::Float32>("/downward_target_debug/camera_u0", 1);
                    static ros::Publisher pub_camera_v0 = node.advertise<std_msgs::Float32>("/downward_target_debug/camera_v0", 1);
                    static ros::Publisher pub_cam_imu_rx = node.advertise<std_msgs::Float32>("/downward_target_debug/cam_imu_rx", 1);
                    static ros::Publisher pub_cam_imu_ry = node.advertise<std_msgs::Float32>("/downward_target_debug/cam_imu_ry", 1);
                    static ros::Publisher pub_cam_imu_rz = node.advertise<std_msgs::Float32>("/downward_target_debug/cam_imu_rz", 1);
                    static ros::Publisher pub_cam_imu_tx = node.advertise<std_msgs::Float32>("/downward_target_debug/cam_imu_tx", 1);
                    static ros::Publisher pub_cam_imu_ty = node.advertise<std_msgs::Float32>("/downward_target_debug/cam_imu_ty", 1);
                    static ros::Publisher pub_cam_imu_tz = node.advertise<std_msgs::Float32>("/downward_target_debug/cam_imu_tz", 1);
                    static ros::Publisher pub_r_g = node.advertise<std_msgs::Float32>("/downward_target_debug/r_g", 1);
                    static ros::Publisher pub_r_b = node.advertise<std_msgs::Float32>("/downward_target_debug/r_b", 1);
                    static ros::Publisher pub_r_n = node.advertise<std_msgs::Float32>("/downward_target_debug/r_n", 1);
                    static ros::Publisher pub_g_r = node.advertise<std_msgs::Float32>("/downward_target_debug/g_r", 1);
                    static ros::Publisher pub_g_b = node.advertise<std_msgs::Float32>("/downward_target_debug/g_b", 1);
                    static ros::Publisher pub_g_n = node.advertise<std_msgs::Float32>("/downward_target_debug/g_n", 1);

                    { std_msgs::Float32 msg; msg.data = camera_f; pub_camera_f.publish(msg); }
                    { std_msgs::Float32 msg; msg.data = camera_u0; pub_camera_u0.publish(msg); }
                    { std_msgs::Float32 msg; msg.data = camera_v0; pub_camera_v0.publish(msg); }
                    { std_msgs::Float32 msg; msg.data = cam_imu_rx; pub_cam_imu_rx.publish(msg); }
                    { std_msgs::Float32 msg; msg.data = cam_imu_ry; pub_cam_imu_ry.publish(msg); }
                    { std_msgs::Float32 msg; msg.data = cam_imu_rz; pub_cam_imu_rz.publish(msg); }
                    { std_msgs::Float32 msg; msg.data = cam_imu_tx; pub_cam_imu_tx.publish(msg); }
                    { std_msgs::Float32 msg; msg.data = cam_imu_ty; pub_cam_imu_ty.publish(msg); }
                    { std_msgs::Float32 msg; msg.data = cam_imu_tz; pub_cam_imu_tz.publish(msg); }
                    { std_msgs::Float32 msg; msg.data = r_g; pub_r_g.publish(msg); }
                    { std_msgs::Float32 msg; msg.data = r_b; pub_r_b.publish(msg); }
                    { std_msgs::Float32 msg; msg.data = r_n; pub_r_n.publish(msg); }
                    { std_msgs::Float32 msg; msg.data = g_r; pub_g_r.publish(msg); }
                    { std_msgs::Float32 msg; msg.data = g_b; pub_g_b.publish(msg); }
                    { std_msgs::Float32 msg; msg.data = g_n; pub_g_n.publish(msg); }
                }
            }
            End();
        }

        BeginMainMenuBar();
        RadioButton("Default view", &mode, mode_see_tracks); SameLine();
        RadioButton("Calibrate camera", &mode, mode_camera_calibration); SameLine();
        RadioButton("Calibrate color", &mode, mode_color_calibration); SameLine();
        if (SmallButton("Snapshot"))
        {
            static int suffix = 0;
            if (latest_image.jpg_data)
            {
                char filename[1024];
                sprintf(filename, "snapshot%04d.jpg", suffix);
                FILE *f = fopen(filename, "wb+");
                fwrite(latest_image.jpg_data, latest_image.jpg_size, 1, f);
                fclose(f);
            }
            {
                downward_target_tracker::info msg = latest_info;
                char filename[1024];
                sprintf(filename, "snapshot%04d.txt", suffix);
                FILE *f = fopen(filename, "w+");
                fprintf(f, "camera_f = %f\n", msg.camera_f);
                fprintf(f, "camera_u0 = %f\n", msg.camera_u0);
                fprintf(f, "camera_v0 = %f\n", msg.camera_v0);
                fprintf(f, "cam_imu_rx = %f\n", msg.cam_imu_rx);
                fprintf(f, "cam_imu_ry = %f\n", msg.cam_imu_ry);
                fprintf(f, "cam_imu_rz = %f\n", msg.cam_imu_rz);
                fprintf(f, "cam_imu_tx = %f\n", msg.cam_imu_tx);
                fprintf(f, "cam_imu_ty = %f\n", msg.cam_imu_ty);
                fprintf(f, "cam_imu_tz = %f\n", msg.cam_imu_tz);
                fprintf(f, "r_g = %f\n", msg.r_g);
                fprintf(f, "r_b = %f\n", msg.r_b);
                fprintf(f, "r_n = %f\n", msg.r_n);
                fprintf(f, "g_r = %f\n", msg.g_r);
                fprintf(f, "g_b = %f\n", msg.g_b);
                fprintf(f, "g_n = %f\n", msg.g_n);
                fprintf(f, "camera_w = %d\n", msg.camera_w);
                fprintf(f, "camera_h = %d\n", msg.camera_h);
                fprintf(f, "imu_rx = %f\n", msg.imu_rx);
                fprintf(f, "imu_ry = %f\n", msg.imu_ry);
                fprintf(f, "imu_rz = %f\n", msg.imu_rz);
                fprintf(f, "imu_tx = %f\n", msg.imu_tx);
                fprintf(f, "imu_ty = %f\n", msg.imu_ty);
                fprintf(f, "imu_tz = %f\n", msg.imu_tz);
                fprintf(f, "image_x = %d\n", msg.image_x);
                fprintf(f, "image_y = %d\n", msg.image_y);
                fclose(f);
            }
            suffix++;
        }
        EndMainMenuBar();

        ros::spinOnce();
    }
    VDBE();

    return 0;
}
