#include "vdb/vdb.h"
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <downward_target_tracker/image.h>
#include <downward_target_tracker/info.h>
#include <downward_target_tracker/tracks.h>
#include <turbojpeg.h>
#include "so_math.h"

struct latest_image_t
{
    unsigned char *jpg_data;
    unsigned int jpg_size;
    unsigned char *I;
    int Ix;
    int Iy;
};

#include "view_rectify.cpp"
#include "view_tracks.cpp"
#include "view_color.cpp"

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
    ros::Publisher pub_selected = node.advertise<std_msgs::Int32>("/downward_target_debug/selected", 1);

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
            view_tracks(latest_info, selected_id);

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

            Begin("Timing");
            {
                Columns(2, "columns_timing");
                Text("Output rate");  NextColumn(); Text("%.2f Hz", 1.0f/latest_info.dt_cycle);            NextColumn(); Separator();
                Text("Frame rate");   NextColumn(); Text("%.2f Hz", 1.0f/latest_info.dt_frame);            NextColumn(); Separator();
                Text("MJPEG to RGB"); NextColumn(); Text("%.2f ms", 1000.0f*latest_info.dt_jpeg_to_rgb);   NextColumn(); Separator();
                Text("Tracker");      NextColumn(); Text("%.2f ms", 1000.0f*latest_info.dt_track_targets); NextColumn(); Separator();
                Columns(1);

                if (latest_info.dt_cycle > 1.25f*1.0f/60.0f)
                    TextColored(ImVec4(1.0f,0.3f,0.1f,1.0f), "Output rate is way less than 60 Hz (message me on Slack).");
            }
            End();
        }

        if (latest_image.I && mode == mode_color_calibration)
        {
            view_color(latest_image, latest_info);
        }

        if (latest_image.I && mode == mode_camera_calibration)
        {
            view_rectify(latest_image, latest_info);
        }

        if (latest_image.I && (mode == mode_color_calibration || mode == mode_camera_calibration))
        {
            Begin("Parameters");
            {
                static float camera_f = latest_info.camera_f;
                static float camera_u0 = latest_info.camera_u0;
                static float camera_v0 = latest_info.camera_v0;
                static float cam_imu_rx = latest_info.cam_imu_rx;
                static float cam_imu_ry = latest_info.cam_imu_ry;
                static float cam_imu_rz = latest_info.cam_imu_rz;
                static float cam_imu_tx = latest_info.cam_imu_tx;
                static float cam_imu_ty = latest_info.cam_imu_ty;
                static float cam_imu_tz = latest_info.cam_imu_tz;
                static float r_g = latest_info.r_g;
                static float r_b = latest_info.r_b;
                static float r_n = latest_info.r_n;
                static float g_r = latest_info.g_r;
                static float g_b = latest_info.g_b;
                static float g_n = latest_info.g_n;
                Text("Hold ALT key to adjust slowly");
                if (CollapsingHeader("Camera intrinsics"))
                {
                    DragFloat("camera_f", &camera_f);
                    DragFloat("camera_u0", &camera_u0);
                    DragFloat("camera_v0", &camera_v0);
                }
                if (CollapsingHeader("Camera extrinsics"))
                {
                    Text("Euler angles defining rotation from camera coordinates to imu coordinates");
                    cam_imu_rx *= 180.0f/3.14f; DragFloat("cam_imu_rx (deg)", &cam_imu_rx); cam_imu_rx *= 3.14f/180.0f;
                    cam_imu_ry *= 180.0f/3.14f; DragFloat("cam_imu_ry (deg)", &cam_imu_ry); cam_imu_ry *= 3.14f/180.0f;
                    cam_imu_rz *= 180.0f/3.14f; DragFloat("cam_imu_rz (deg)", &cam_imu_rz); cam_imu_rz *= 3.14f/180.0f;
                    Text("Camera center relative imu center in imu coordinates");
                    cam_imu_tx *= 100.0f; DragFloat("cam_imu_tx (cm)", &cam_imu_tx); cam_imu_tx /= 100.0f;
                    cam_imu_ty *= 100.0f; DragFloat("cam_imu_ty (cm)", &cam_imu_ty); cam_imu_ty /= 100.0f;
                    cam_imu_tz *= 100.0f; DragFloat("cam_imu_tz (cm)", &cam_imu_tz); cam_imu_tz /= 100.0f;
                }
                if (CollapsingHeader("Red thresholds"))
                {
                    SliderFloat("red over green (r_g)", &r_g, 1.0f, 10.0f);
                    SliderFloat("red over blue (r_b)", &r_b, 1.0f, 10.0f);
                    SliderFloat("minimum brightness (r_n)", &r_n, 0.0f, 255.0f);
                }
                if (CollapsingHeader("Green thresholds"))
                {
                    SliderFloat("green over red (g_r)", &g_r, 1.0f, 10.0f);
                    SliderFloat("green over blue (g_b)", &g_b, 1.0f, 10.0f);
                    SliderFloat("minimum brightness (g_n)", &g_n, 0.0f, 255.0f);
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
            End();
        }

        BeginMainMenuBar();
        RadioButton("Default view", &mode, mode_see_tracks); SameLine();
        RadioButton("Calibrate camera", &mode, mode_camera_calibration); SameLine();
        RadioButton("Calibrate color", &mode, mode_color_calibration); SameLine();
        if (SmallButton("Take a snapshot"))
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
        SameLine();
        if (selected_id >= 0)
            Text("Publishing %d", selected_id);
        EndMainMenuBar();

        {
            std_msgs::Int32 msg;
            msg.data = selected_id;
            pub_selected.publish(msg);
        }

        ros::spinOnce();
    }
    VDBE();

    return 0;
}
