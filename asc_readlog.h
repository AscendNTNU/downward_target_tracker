#include <stdio.h>
#include <stdint.h>
#include <assert.h>

struct log_entry_t
{
    int video_i;
    float qx;
    float qy;
    float qz;
    float qw;
    float r;
};

static log_entry_t log_entries[LOG_MAX_COUNT] = {0};
static int         log_count = 0;

void log_readData()
{
    FILE *file_imu   = fopen(IMU_LOG, "r");
    FILE *file_lidar = fopen(LIDAR_LOG, "r");
    FILE *file_video = fopen(VIDEO_LOG, "r");

    assert(file_imu);
    assert(file_lidar);
    assert(file_video);

    float latest_imu_qx;
    float latest_imu_qy;
    float latest_imu_qz;
    float latest_imu_qw;
    uint64_t latest_imu_t;
    {
        uint64_t t;
        float qx, qy, qz, qw;
        float wx, wy, wz;
        float ax, ay, az;
        assert(11 == fscanf(file_imu, "%f %f %f %f %f %f %f %f %f %f %llu", &qx, &qy, &qz, &qw, &wx, &wy, &wz, &ax, &ay, &az, &t));
        latest_imu_qx = qx;
        latest_imu_qy = qy;
        latest_imu_qz = qz;
        latest_imu_qw = qw;
        latest_imu_t = t;
    }

    float latest_lidar_r;
    uint64_t latest_lidar_t;
    {
        uint64_t t;
        float r;
        assert(2 == fscanf(file_lidar, "%f %llu", &r, &t));
        latest_lidar_r = r;
        latest_lidar_t = t;
    }

    log_count = 0;

    while (log_count < LOG_MAX_COUNT)
    {
        int video_i;
        uint64_t video_t;
        if (2 != fscanf(file_video, "%d %llu", &video_i, &video_t))
            return;

        {
            uint64_t t = latest_lidar_t;
            float r = latest_lidar_r;
            while (t < video_t)
            {
                if (2 != fscanf(file_lidar, "%f %llu", &r, &t))
                    return;
                latest_lidar_t = t;
                latest_lidar_r = r;
            }
        }

        {
            uint64_t t = latest_imu_t;
            float qx = latest_imu_qx;
            float qy = latest_imu_qy;
            float qz = latest_imu_qz;
            float qw = latest_imu_qw;
            float wx, wy, wz;
            float ax, ay, az;
            while (t < video_t)
            {
                if (11 != fscanf(file_imu, "%f %f %f %f %f %f %f %f %f %f %llu", &qx, &qy, &qz, &qw, &wx, &wy, &wz, &ax, &ay, &az, &t))
                    return;
                latest_imu_t = t;
                latest_imu_qx = qx;
                latest_imu_qy = qy;
                latest_imu_qz = qz;
                latest_imu_qw = qw;
            }
        }

        log_entries[log_count].video_i = video_i;
        log_entries[log_count].qx = latest_imu_qx;
        log_entries[log_count].qy = latest_imu_qy;
        log_entries[log_count].qz = latest_imu_qz;
        log_entries[log_count].qw = latest_imu_qw;
        log_entries[log_count].r = latest_lidar_r;
        log_count++;
    }

    fclose(file_imu);
    fclose(file_lidar);
    fclose(file_video);
}
