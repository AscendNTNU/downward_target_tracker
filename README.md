### Contents

1. Compiling
2. Running the tracker and the list of targets
3. Setting camera controls
4. Verifying that things are working
    1. Calibrating color thresholds
    2. Calibrating fisheye parameters
    3. Calibrating camera/imu mounting

### Compiling

Get the video 4 linux 2 development libraries (v4l2)
```
$ sudo apt-get install libv4l-dev
$ sudo apt-get install v4l-utils
```

Get the turbojpeg library (see [libjpeg-turbo/BUILDING.md](https://github.com/libjpeg-turbo/libjpeg-turbo/blob/master/BUILDING.md))
```
$ git clone https://github.com/libjpeg-turbo/libjpeg-turbo
$ cd libjpeg-turbo
$ autoreconf -fiv
$ mkdir build
$ cd build
$ sh ../configure
$ make
$ make install prefix=/usr/local libdir=/usr/local/lib64
```

Get SDL2 (for the debugger)
```
$ sudo apt-get install libsdl2-dev
```

### Run the tracker

Step 1) Set the parameters (camera path, drone pose topic, desired image and info publish time delays, and camera and color calibration). See bottom of this document for a description of the parameters and where to find them.

Step 2)

```
$ rosrun downward_target_tracker tracker
```

This will run the executable compiled from ```main_drone.cpp```.  A list of tracked targets is published 60 times per second in the topic ```downward_target_tracker/tracks``` (see [```tracks.msg```](msg/tracks.msg)). The message data can be used like this:

```
downward_target_tracker::tracks msg // From callback
int selected_id = 1234; // Planning gives us this, or manually with the debugger (below)
for (int i = 0; i < msg.num_targets)
{
    int id = msg.unique_id[i];
    float x = msg.position_x[i];
    float y = msg.position_y[i];
    float vx = msg.velocity_x[i];
    float vy = msg.velocity_y[i];

    if (id == selected_id)
    {
        // publish x,y,vx,vy to velocity-feedforward controller node????
    }
}
```

### Setting camera controls

```set_camera_controls.sh``` sets camera exposure, gain, powerline frequency, ..., to some hardcoded values. **You must change these values during testing** depending on lighting. Make sure that the framerate does not dip below 60 (see Timing window in debugger).

### Make sure everything works

Compile and run the tracker.
```
$ rosrun downward_target_tracker tracker
```

Run the debugger (on your computer).
```
$ rosrun downward_target_tracker debugger
```

With the main tab open in the debugger, if you are

* looking at a target (red or green plate)
* and you are either standing ONE METER above the ground at zero pitch and roll,
* or you are publishing the drone pose,

you should see a non-empty list of targets, and a bounding box around the target. Clicking one of the entries will highlight it in red, and its ID will be published at ```downward_target_debug/selected```.

If not, we need to calibrate intrinsics, extrinsics (camera mounting point), or color thresholds.

If you need help, click the "Take a snapshot" button and send the files (snapshot*.jpg snapshot*.txt created in the directory you ran the debugger) to me.

#### Calibrate color

1. Keep a red and green target plate in view.
2. Click the "calibrate color" tab in debugger.
3. Take snapshot and send to me, or adjust 'red' and 'green' thresholds until enough pixels are highlighted on both plates with as few outliers.
4. Save the thresholds (see below).

#### Calibrate camera intrinsics

With the "calibrate camera" tab open:

1. Point the camera at a [checkerboard](http://docs.opencv.org/2.4/_downloads/pattern.png)
2. Take a snapshot and send to me; or keep the camera at a known rotation and translation from the checkerboard and try to adjust f, u0 and v0.
3. Save parameters (see below).

#### Calibrate camera extrinsics

These define how the camera is rotated and translated relative to the IMU coordinate frame.

If the only difference between them is a rotation about z, then you can set that directly with ```CAM_IMU_R_Z_INIT``` (see below).

Verify that things look correct by opening "calibrate camera" tab.

1. Look at a grid pattern.
2. Align drone axes with grid axes.
3. Tilt the drone in either x or y axis, and verify that the visualized grid pattern matches with real-life.

#### Parameters
Open ```main_drone.cpp``` and look at the top of the file. Change the parameters if you need to:

Parameter   | What
------------|-----
DUMMY_IMAGE | 0 will use USB camera; 1 will use a static image embedded in source code
DEVICE_NAME | i.e. /dev/video1
IMU_POSE_TOPIC | Topic on which drone pose relative grid is published
INFO_PUBLISH_INTERVAL | Change this to limit how often debug visualization info is sent
IMAGE_PUBLISH_INTERVAL | Change this to limit how often compressed camera image is sent (default is every frame)
CAM_IMU_RX/Y/Z_INIT | Euler angles defining rotation from camera coordinates to imu coordinates
CAM_IMU_TX/Y/Z_INIT | Camera center relative imu center in imu coordinates
R_G/B/N_INIT | Color thresholds for 'red' classification (minimum red/green ratio, minimum red/blue ratio, and minimum average brightness)
G_R/B/N_INIT | Color thresholds for 'green' classification (minimum green/red ratio, minimum green/blue ratio, and minimum average brightness)
CAMERA_WIDTH/HEIGHT | Request video resolution from USB camera
CAMERA_BUFFERS | This might need to be changed if output rate (see debugger) is less than 60 Hz
CAMERA_LEVELS | Don't change?
CAMERA_F/U0/V0_INIT | Fisheye projection parameters, depends on video resolution
