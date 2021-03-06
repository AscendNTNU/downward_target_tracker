#define VDB_DISABLE_PROTIP
#include "parameters.h"
#include "vdb/vdb.h"
#include "vdb/vdb_imgui_draw.h"
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <downward_target_tracker/image.h>
#include <downward_target_tracker/info.h>
#include <downward_target_tracker/tracks.h>
#include <ascend_msgs/LineCounter.h>
#include <turbojpeg.h>
#include "so_math.h"

#include <cstdio>

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
#include "view_threshold.cpp"
#include "view_line_counter.cpp"

latest_image_t latest_image = {0};
downward_target_tracker::info latest_info;
downward_target_tracker::tracks latest_tracks;
ascend_msgs::LineCounter latest_line_counter;

void callback_info(downward_target_tracker::info msg) { latest_info = msg; }
void callback_tracks(downward_target_tracker::tracks msg) { latest_tracks = msg; }
void callback_line_counter(ascend_msgs::LineCounter msg) { latest_line_counter = msg; }
void callback_image(downward_target_tracker::image msg)
{
    int subsamp,width,height,error;
    unsigned char *jpg_data = (unsigned char*)&msg.jpg_data[0];
    unsigned int jpg_size = (unsigned int)msg.jpg_data.size();

    static tjhandle decompressor = tjInitDecompress();

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
    latest_image.Ix = width>>CAMERA_LEVELS;
    latest_image.Iy = height>>CAMERA_LEVELS;

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
    ros::Subscriber sub_image        = node.subscribe(IMAGE_TOPIC, 1, callback_image);
    ros::Subscriber sub_info         = node.subscribe(INFO_TOPIC, 1, callback_info);
    ros::Subscriber sub_tracks       = node.subscribe(TRACKS_TOPIC, 1, callback_tracks);
    ros::Subscriber sub_line_counter = node.subscribe(LINE_COUNTER_TOPIC, 1, callback_line_counter);
    ros::Publisher pub_selected      = node.advertise<std_msgs::Int32>(SELECTED_TOPIC, 1);

    int selected_id = -1;

    VDBB("downward_target_debug");
    {
        const int mode_see_tracks = 0;
        const int mode_camera_calibration = 1;
        const int mode_color_calibration = 2;
        const int mode_white_threshold = 3;
        const int mode_line_counter = 4;
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

            SetNextWindowSize(ImVec2(400, 300), ImGuiSetCond_Appearing);
            Begin("Select target");
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

            SetNextWindowSize(ImVec2(400, 300), ImGuiSetCond_Appearing);
            Begin("Timing");
            {
                static bool freeze = false;
                Checkbox("Freeze", &freeze);
                #define PlotTiming(LABEL, LATEST, MIN, MAX)                \
                {                                                            \
                    const int count = 60*3;                                  \
                    static float graph[count];                               \
                    if (!freeze) {                                           \
                    for (int i = 0; i < count-1; i++)                        \
                        graph[i] = graph[i+1]; }                             \
                    graph[count-1] = LATEST;                                 \
                    PlotLines(LABEL, graph, count, 0, NULL, MIN, MAX, ImVec2(200,0)); \
                }

                Text("Tracker");
                PlotTiming("Output rate (Hz)",  1.0f/latest_info.dt_cycle, 0.0f, 60.0f); SameLine(); Text("%.2f Hz", 1.0f/latest_info.dt_cycle);
                PlotTiming("Frame rate (Hz)",   1.0f/latest_info.dt_frame, 0.0f, 60.0f); SameLine(); Text("%.2f Hz", 1.0f/latest_info.dt_frame);
                PlotTiming("MJPEG to RGB (ms)" , 1000.0f*latest_info.dt_jpeg_to_rgb, 0.0f, 10.0f); SameLine(); Text("%.2f ms", 1000.0f*latest_info.dt_jpeg_to_rgb);
                PlotTiming("Tracker (ms)",      1000.0f*latest_info.dt_track_targets, 0.0f, 10.0f); SameLine(); Text("%.2f ms", 1000.0f*latest_info.dt_track_targets);
                Text("Line counter");
                PlotTiming("MJPEG to RGB (ms)##line_counter", 1000.0f*latest_info.line_counter_dt_jpeg_to_rgb, 0.0f, 10.0f); SameLine(); Text("%.2f ms", 1000.0f*latest_info.line_counter_dt_jpeg_to_rgb);
                PlotTiming("Threshold (ms)##line_counter",    1000.0f*latest_info.line_counter_dt_threshold, 0.0f, 10.0f); SameLine(); Text("%.2f ms", 1000.0f*latest_info.line_counter_dt_threshold);
                PlotTiming("Find grid (ms)##line_counter",    1000.0f*latest_info.line_counter_dt_find_grid, 0.0f, 30.0f); SameLine(); Text("%.2f ms", 1000.0f*latest_info.line_counter_dt_find_grid);

                if (latest_info.dt_cycle > 1.25f*1.0f/60.0f)
                {
                    PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.3f, 0.1f, 1.0f));
                    TextWrapped("Output rate is way less than 60 Hz (message me on Slack).");
                    PopStyleColor();
                }
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

        if (latest_image.I && mode == mode_white_threshold)
        {
            view_threshold(latest_image, latest_info);
        }

        if (latest_image.I && mode == mode_line_counter)
        {
            view_line_counter(latest_image, latest_info, latest_line_counter);
        }

        if (latest_image.I &&
            (mode == mode_color_calibration ||
             mode == mode_camera_calibration ||
             mode == mode_white_threshold ||
             mode == mode_line_counter))
        {
            SetNextWindowSize(ImVec2(400, 300), ImGuiSetCond_Appearing);
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
                static float white_threshold_r = latest_info.white_threshold_r;
                static float white_threshold_g = latest_info.white_threshold_g;
                static float white_threshold_b = latest_info.white_threshold_b;
                static float white_threshold_d = latest_info.white_threshold_d;
                static float pinhole_fov_x = latest_info.pinhole_fov_x;
                static float sobel_threshold = latest_info.sobel_threshold;
                static float maxima_threshold = latest_info.maxima_threshold;
                static float max_error = latest_info.max_error;
                static float tile_width = latest_info.tile_width;
                Text("Hold ALT key to adjust slowly");
                if (mode == mode_camera_calibration && CollapsingHeader("Camera intrinsics"))
                {
                    DragFloat("camera_f", &camera_f);
                    DragFloat("camera_u0", &camera_u0);
                    DragFloat("camera_v0", &camera_v0);
                }
                if (mode == mode_camera_calibration && CollapsingHeader("Camera extrinsics"))
                {
                    TextWrapped("Euler angles defining rotation from camera coordinates to imu coordinates");
                    cam_imu_rx *= 180.0f/3.14f; DragFloat("cam_imu_rx (deg)", &cam_imu_rx); cam_imu_rx *= 3.14f/180.0f;
                    cam_imu_ry *= 180.0f/3.14f; DragFloat("cam_imu_ry (deg)", &cam_imu_ry); cam_imu_ry *= 3.14f/180.0f;
                    cam_imu_rz *= 180.0f/3.14f; DragFloat("cam_imu_rz (deg)", &cam_imu_rz); cam_imu_rz *= 3.14f/180.0f;
                    TextWrapped("Camera center relative imu center in imu coordinates");
                    cam_imu_tx *= 100.0f; DragFloat("cam_imu_tx (cm)", &cam_imu_tx); cam_imu_tx /= 100.0f;
                    cam_imu_ty *= 100.0f; DragFloat("cam_imu_ty (cm)", &cam_imu_ty); cam_imu_ty /= 100.0f;
                    cam_imu_tz *= 100.0f; DragFloat("cam_imu_tz (cm)", &cam_imu_tz); cam_imu_tz /= 100.0f;
                }
                if (mode == mode_color_calibration && CollapsingHeader("Red thresholds"))
                {
                    SliderFloat("red over green (r_g)", &r_g, 1.0f, 10.0f);
                    SliderFloat("red over blue (r_b)", &r_b, 1.0f, 10.0f);
                    SliderFloat("minimum brightness (r_n)", &r_n, 0.0f, 255.0f);
                }
                if (mode == mode_color_calibration && CollapsingHeader("Green thresholds"))
                {
                    SliderFloat("green over red (g_r)", &g_r, 1.0f, 10.0f);
                    SliderFloat("green over blue (g_b)", &g_b, 1.0f, 10.0f);
                    SliderFloat("minimum brightness (g_n)", &g_n, 0.0f, 255.0f);
                }
                if (mode == mode_white_threshold && CollapsingHeader("White threshold (grid detector)"))
                {
                    SliderFloat("white_threshold_r", &white_threshold_r, 0.0f, 255.0f);
                    SliderFloat("white_threshold_g", &white_threshold_g, 0.0f, 255.0f);
                    SliderFloat("white_threshold_b", &white_threshold_b, 0.0f, 255.0f);
                    SliderFloat("white_threshold_d", &white_threshold_d, 0.0f, 255.0f);
                }
                if (mode == mode_line_counter && CollapsingHeader("Tuning values (grid detector)"))
                {
                    pinhole_fov_x *= 180.0f/3.14f; SliderFloat("pinhole_fov_x (deg)", &pinhole_fov_x, 45.0f, 180.0f); pinhole_fov_x *= 3.14f/180.0f;
                    SliderFloat("sobel_threshold", &sobel_threshold, 0.0f, 100.0f);
                    SliderFloat("maxima_threshold", &maxima_threshold, 0.0f, 100.0f);
                    SliderFloat("max_error", &max_error, 0.0f, 1.0f);
                    SliderFloat("tile_width (m)", &tile_width, 0.1f, 5.0f);
                }

                static ros::Publisher pub_camera_f          = node.advertise<std_msgs::Float32>("/target_debug/camera_f", 1);
                static ros::Publisher pub_camera_u0         = node.advertise<std_msgs::Float32>("/target_debug/camera_u0", 1);
                static ros::Publisher pub_camera_v0         = node.advertise<std_msgs::Float32>("/target_debug/camera_v0", 1);
                static ros::Publisher pub_cam_imu_rx        = node.advertise<std_msgs::Float32>("/target_debug/cam_imu_rx", 1);
                static ros::Publisher pub_cam_imu_ry        = node.advertise<std_msgs::Float32>("/target_debug/cam_imu_ry", 1);
                static ros::Publisher pub_cam_imu_rz        = node.advertise<std_msgs::Float32>("/target_debug/cam_imu_rz", 1);
                static ros::Publisher pub_cam_imu_tx        = node.advertise<std_msgs::Float32>("/target_debug/cam_imu_tx", 1);
                static ros::Publisher pub_cam_imu_ty        = node.advertise<std_msgs::Float32>("/target_debug/cam_imu_ty", 1);
                static ros::Publisher pub_cam_imu_tz        = node.advertise<std_msgs::Float32>("/target_debug/cam_imu_tz", 1);
                static ros::Publisher pub_r_g               = node.advertise<std_msgs::Float32>("/target_debug/r_g", 1);
                static ros::Publisher pub_r_b               = node.advertise<std_msgs::Float32>("/target_debug/r_b", 1);
                static ros::Publisher pub_r_n               = node.advertise<std_msgs::Float32>("/target_debug/r_n", 1);
                static ros::Publisher pub_g_r               = node.advertise<std_msgs::Float32>("/target_debug/g_r", 1);
                static ros::Publisher pub_g_b               = node.advertise<std_msgs::Float32>("/target_debug/g_b", 1);
                static ros::Publisher pub_g_n               = node.advertise<std_msgs::Float32>("/target_debug/g_n", 1);
                static ros::Publisher pub_white_threshold_r = node.advertise<std_msgs::Float32>("/target_debug/white_threshold_r", 1);
                static ros::Publisher pub_white_threshold_g = node.advertise<std_msgs::Float32>("/target_debug/white_threshold_g", 1);
                static ros::Publisher pub_white_threshold_b = node.advertise<std_msgs::Float32>("/target_debug/white_threshold_b", 1);
                static ros::Publisher pub_white_threshold_d = node.advertise<std_msgs::Float32>("/target_debug/white_threshold_d", 1);
                static ros::Publisher pub_pinhole_fov_x     = node.advertise<std_msgs::Float32>("/target_debug/pinhole_fov_x", 1);
                static ros::Publisher pub_sobel_threshold   = node.advertise<std_msgs::Float32>("/target_debug/sobel_threshold", 1);
                static ros::Publisher pub_maxima_threshold  = node.advertise<std_msgs::Float32>("/target_debug/maxima_threshold", 1);
                static ros::Publisher pub_max_error         = node.advertise<std_msgs::Float32>("/target_debug/max_error", 1);
                static ros::Publisher pub_tile_width        = node.advertise<std_msgs::Float32>("/target_debug/tile_width", 1);

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
                { std_msgs::Float32 msg; msg.data = white_threshold_r; pub_white_threshold_r.publish(msg); }
                { std_msgs::Float32 msg; msg.data = white_threshold_g; pub_white_threshold_g.publish(msg); }
                { std_msgs::Float32 msg; msg.data = white_threshold_b; pub_white_threshold_b.publish(msg); }
                { std_msgs::Float32 msg; msg.data = white_threshold_d; pub_white_threshold_d.publish(msg); }
                { std_msgs::Float32 msg; msg.data = pinhole_fov_x; pub_pinhole_fov_x.publish(msg); }
                { std_msgs::Float32 msg; msg.data = sobel_threshold; pub_sobel_threshold.publish(msg); }
                { std_msgs::Float32 msg; msg.data = maxima_threshold; pub_maxima_threshold.publish(msg); }
                { std_msgs::Float32 msg; msg.data = max_error; pub_max_error.publish(msg); }
                { std_msgs::Float32 msg; msg.data = tile_width; pub_tile_width.publish(msg); }
            }
            End();
        }

        BeginMainMenuBar();
        RadioButton("Default view", &mode, mode_see_tracks); SameLine();
        RadioButton("Calibrate camera", &mode, mode_camera_calibration); SameLine();
        RadioButton("Calibrate color", &mode, mode_color_calibration); SameLine();
        RadioButton("White threshold", &mode, mode_white_threshold); SameLine();
        RadioButton("Grid detector", &mode, mode_line_counter); SameLine();
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
                fprintf(f, "white_threshold_r = %f\n", msg.white_threshold_r);
                fprintf(f, "white_threshold_g = %f\n", msg.white_threshold_g);
                fprintf(f, "white_threshold_b = %f\n", msg.white_threshold_b);
                fprintf(f, "white_threshold_d = %f\n", msg.white_threshold_d);
                fprintf(f, "pinhole_fov_x = %f\n", msg.pinhole_fov_x);
                fprintf(f, "sobel_threshold = %f\n", msg.sobel_threshold);
                fprintf(f, "maxima_threshold = %f\n", msg.maxima_threshold);
                fprintf(f, "max_error = %f\n", msg.max_error);
                fprintf(f, "tile_width = %f\n", msg.tile_width);
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
