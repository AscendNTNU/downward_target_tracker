### How do I compile you?

Get the video 4 linux 2 development libraries (v4l2)
```
$ sudo apt-get install libv4l-dev
$ sudo apt-get install v4l-utils
```

Get the turbojpeg library
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

See [libjpeg-turbo/BUILDING.md](https://github.com/libjpeg-turbo/libjpeg-turbo/blob/master/BUILDING.md) if that didn't work.

(For the debugger; not on the drone) Get SDL2
```
$ sudo apt-get install libsdl2-dev
```

### Run the tracker

```
$ rosrun downward_target_tracker tracker
```

This will run the executable compiled from ```main_drone.cpp```.  A list of tracked targets is published 60 times per second in the topic ```downward_target_tracker/tracks```. See [```tracks.msg```](msg/tracks.msg). The message data can be used like this:

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

### Make sure everything isn't broken

Open ```main_drone.cpp``` and look at the top of the file. Change the parameters if you need to:
* **DUMMY_IMAGE**: 0 - use camera, 1 - use a static image (embedded in source code)
* **DEVICE_NAME**: Downward camera path
* **IMU_POSE_TOPIC**: Topic on which the drone pose (relative grid) is published
* **INFO_PUBLISH_INTERVAL**: Change this to limit how often debug visualization info is sent
* **IMAGE_PUBLISH_INTERVAL**: Change this to limit how often camera image (compressed) is sent
* **CAM_IMU_R..._INIT**: These define the rotation from camera coordinates to imu coordinates
* **CAM_IMU_T..._INIT**: These define the camera center relative imu center, in imu coordinates
* **R_..._INIT**: Color thresholds for 'red' classification
* **G_..._INIT**: Color thresholds for 'green' classification
* **CAMERA_WIDTH,HEIGHT**: Requested video resolution from usb camera
* **CAMERA_BUFFERS**: Change this if video feed is choppy (see ```asc_usbcam.h```).
* **CAMERA_LEVELS**: Don't change?
* **CAMERA_F,U0,V0_INIT**: Fisheye projection parameters. Change if you change video resolution.

Compile and run the tracker.
```
$ rosrun downward_target_tracker tracker
```

Run the debugger (on your computer).
```
$ rosrun downward_target_tracker debugger
```

#### Testing if detector and tracker works

If you are

* looking at a target (red or green plate)
* and you have the main tab open in the debugger,
* and you are either standing ONE METER above the ground,
* or you are publishing the drone pose,
* and the pitch and roll are both zero in either case

then you should see a non-empty list of targets, and a bounding box around the target. If you click at one of the entries, it will be highlighted (red color), and its unique id will be published at ```downward_target_debug/selected```.

If not, we need to calibrate intrinsics, extrinsics (camera mounting point), or color thresholds.

If you need help, click the "Take a snapshot" button and send the files (snapshot*.jpg snapshot*.txt created in the directory you ran the debugger) to me.

#### Calibrate color

1. Keep a red and green target plate in view.
2. Click the "calibrate color" tab in debugger.
3. Take snapshot and send to me, or adjust 'red' and 'green' thresholds until enough pixels are highlighted on both plates with as few outliers.
4. Save the thresholds by updating ```R_G_INIT```, ```R_B_INIT```, ... etc.

#### Calibrate camera intrinsics

With the "calibrate camera" tab open:

1. Point the camera at a [checkerboard](http://docs.opencv.org/2.4/_downloads/pattern.png)
2. Take a snapshot and send to me; or keep the camera at a known rotation and translation from the checkerboard and try to adjust f, u0 and v0.
3. Save parameters by updating ```CAMERA_F_INIT```, ... etc.

#### Calibrate camera extrinsics

These define how the camera is rotated and translated relative to the IMU coordinate frame.

If the only difference between them is a rotation about z, then you can set that directly with ```CAM_IMU_R_Z_INIT```.

Verify that things look correct by opening "calibrate camera" tab.

1. Look at a grid pattern.
2. Align drone axes with grid axes.
3. Tilt the drone in either x or y axis, and verify that the visualized grid pattern matches with real-life.
