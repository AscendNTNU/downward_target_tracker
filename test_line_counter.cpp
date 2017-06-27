// g++ test_line_counter.cpp -o app -lv4l2 -turbojpeg -lpthread

#define DEVICE_NAME    "/dev/video0"
#define CAMERA_WIDTH   800
#define CAMERA_HEIGHT  600
#define CAMERA_BUFFERS 3

#include "src/asc_usbcam.h"
#include "src/mjpg_to_jpg.h"

#include <stdint.h>
#include <signal.h>
#include <time.h>
#include <pthread.h>
#include <assert.h>

uint64_t getnsec()
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

// These need to be volatile, otherwise the main_line_counter thread did
// not work properly when I compile with optimizations (-O2).
volatile bool line_counter_jpg_available = false;
volatile bool line_counter_using_jpg = false;
volatile unsigned int line_counter_jpg_size;
static unsigned char line_counter_jpg_data[CAMERA_WIDTH*CAMERA_HEIGHT*3];

void *main_line_counter(void *)
{
    printf("line counter!\n");
    for (int frame = 0;; frame++)
    {
        while (!line_counter_jpg_available)
        {
        }

        line_counter_jpg_available = false;
        line_counter_using_jpg = true;

        float dt_fwrite = 0.0f;
        {
            uint64_t t1 = getnsec();
            FILE *f = fopen("test.jpg", "w+");
            assert(f);
            fwrite(line_counter_jpg_data, line_counter_jpg_size, 1, f);
            fclose(f);
            uint64_t t2 = getnsec();
            dt_fwrite = (t2-t1)/1e9;
        }

        usleep(50*1000);

        printf("%d. %.2f ms\n", frame, 1000.0f*dt_fwrite);

        line_counter_using_jpg = false;
    }
}

int main(int, char **)
{
    {
        pthread_t t;
        pthread_create(&t, NULL, main_line_counter, NULL);
    }

    signal(SIGINT, ctrlc);

    {
        usbcam_opt_t opt = {0};
        opt.device_name = DEVICE_NAME;
        opt.pixel_format = V4L2_PIX_FMT_MJPEG;
        opt.width = CAMERA_WIDTH;
        opt.height = CAMERA_HEIGHT;
        opt.buffers = CAMERA_BUFFERS;
        usbcam_init(opt);
    }

    for (int frame = 0;; frame++)
    {
        // RECEIVE LATEST IMAGE
        unsigned char *jpg_data = 0;
        unsigned int jpg_size = 0;
        timeval timestamp = {0};
        usbcam_lock_mjpg(&jpg_data, &jpg_size, &timestamp);

        float dt_memcpy = 0.0f;
        if (!line_counter_using_jpg)
        {
            uint64_t t1 = getnsec();
            memcpy(line_counter_jpg_data, jpg_data, jpg_size);
            line_counter_jpg_size = jpg_size;
            line_counter_jpg_available = true;
            uint64_t t2 = getnsec();
            dt_memcpy = (t2-t1)/1e9;
        }

        // MEASURE TIME BETWEEN WHEN IMAGES WERE TAKEN
        float dt_frame = 0.0f;
        {
            uint64_t sec = (uint64_t)timestamp.tv_sec;
            uint64_t usec = (uint64_t)timestamp.tv_usec;
            uint64_t t = sec*1000*1000 + usec;
            static uint64_t last_t = t;
            dt_frame = (t-last_t)/1e6;
            last_t = t;
            // printf("%d. %.2f ms\t %.2f ms\n", frame, 1000.0f*dt_frame, 1000.0f*dt_memcpy);
        }

        usbcam_unlock();
    }

    return 0;
}
