#### The Line Counter

This repository also contains the line counter node (see [src/main_line_counter](src/main_line_counter.cpp)).

**Its input is**: the latest estimate of the drone's pitch, roll and height. The ROS topic for this input is ```IMU_POSE_TOPIC```, which you can change in [src/parameters.h](src/parameters.h).

**Its output is**: an estimate of the drone's yaw, and x and y coordinate. The ROS topic for this output is ```LINE_COUNTER_TOPIC```, which you can change in [src/parameters.h](src/parameters.h).

**Mathematically**: Let the pose of the drone relative the grid be
```
    R = Rz(yaw)Ry(pitch)Rx(roll)
    T = (x, y, height)
```
where R is the drone frame expressed in grid coordinates; Rz, Ry, Rx are rotation matrices about the z, y and x axes; and T is the center of the drone expressed in grid coordinates. In other words, a coordinate vector in drone coordinates, p, can be transformed to grid coordinates, q,  by ```q = Rp + T```.

Then the grid detector should receive as input ```pitch```, ```roll``` and ```height```, and will give as output ```yaw```, ```x``` and ```y```.

However, there are four possibilities for ```yaw```, since we cannot from a single image of a grid tile determine which axis is the x axis (the red line). Therefore, the output consists of four ```yaw``` angles, each offset by 90 degrees, and with its own associated ```x``` and ```y```.

**The ROS message**: Is defined in ascend_msgs/LineCounter.msg. The timestamp is in nanoseconds since the UNIX epoch and is calculated like so:
```
#include <stdint.h> // uint64_t
uint64_t getnsec()
{
    struct timespec ts = {};
    clock_gettime(CLOCK_REALTIME, &ts);
    uint64_t result = ((uint64_t)ts.tv_sec)*1000000000 +
                      ((uint64_t)ts.tv_nsec);
    return result;
}
```


