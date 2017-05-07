#define USBCAM_DEBUG
#define CAMERA_NAME        "/dev/video1"
#define CAMERA_WIDTH       800
#define CAMERA_HEIGHT      600
#define CAMERA_BUFFERS     3
#define CAMERA_LEVELS      2 // Downscale factor (0=none, 1=half, 2=quarter)

#include "asc_usbcam.h"
#include "asc_tracker.h"
#include <time.h>
#include <signal.h>
#include <assert.h>
#include <stdint.h>

#include <ros/ros.h>
#include <downward_target_tracker/calibration.h>
#include <downward_target_tracker/image.h>

uint64_t get_nanoseconds()
{
    struct timespec ts = {};
    clock_gettime(CLOCK_REALTIME, &ts);
    uint64_t result = ((uint64_t)ts.tv_sec)*1000000000 +
                      ((uint64_t)ts.tv_nsec);
    return result;
}

void ctrlc(int)
{
    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "downward_target_tracker");
    ros::NodeHandle node;
    // ros::Subscriber sub;
    // sub = node.subscribe("/", 1, callback_)
    ros::Publisher pub_calibration = node.advertise<downward_target_tracker::calibration>("downward_target_tracker/calibration", 1);
    ros::Publisher pub_image       = node.advertise<downward_target_tracker::image>("downward_target_tracker/image", 1);

    signal(SIGINT, ctrlc);

    usbcam_opt_t opt = {0};
    opt.device_name = CAMERA_NAME;
    opt.pixel_format = V4L2_PIX_FMT_MJPEG;
    opt.width = CAMERA_WIDTH;
    opt.height = CAMERA_HEIGHT;
    opt.buffers = CAMERA_BUFFERS;
    usbcam_init(opt);

    float camera_f = 434.0f;
    float camera_u0 = 375.0f;
    float camera_v0 = 275.0f;

    const int Ix = CAMERA_WIDTH>>CAMERA_LEVELS;
    const int Iy = CAMERA_HEIGHT>>CAMERA_LEVELS;
    static unsigned char I[CAMERA_WIDTH*CAMERA_HEIGHT*3];

    for (int i = 0;; i++)
    {
        unsigned char *jpg_data = 0;
        unsigned int jpg_size = 0;
        timeval timestamp = {0};
        usbcam_lock(&jpg_data, &jpg_size, &timestamp);

        float dt_internal = 0.0f;
        {
            uint64_t sec = (uint64_t)timestamp.tv_sec;
            uint64_t usec = (uint64_t)timestamp.tv_usec;
            uint64_t t = sec*1000*1000 + usec;
            static uint64_t last_t = t;
            dt_internal = (t-last_t)/1e6;
            last_t = t;
        }

        float dt_decompress = 0.0f;
        {
            uint64_t t1 = get_nanoseconds();
            if (!usbcam_jpeg_to_rgb(Ix, Iy, I, jpg_data, jpg_size))
            {
                usbcam_unlock();
                continue;
            }
            uint64_t t2 = get_nanoseconds();
            dt_decompress = (t2-t1)/1e9;
        }

        {
            downward_target_tracker::calibration msg;
            msg.camera_f = 1.0f;
            msg.camera_u0 = 2.0f;
            msg.camera_v0 = 3.0f;
            pub_calibration.publish(msg);
        }

        {
            downward_target_tracker::image msg;
            msg.jpg_data.resize(jpg_size);
            memcpy(&msg.jpg_data[0], jpg_data, jpg_size);
            pub_image.publish(msg);
        }

        ros::spinOnce();

        printf("%6.2f ms (decompress)\t%6.2f ms (frame)\n", 1000.0f*dt_decompress, 1000.0f*dt_internal);
        usbcam_unlock();
    }

    return 0;
}
