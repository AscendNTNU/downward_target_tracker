### About this repository

This repository contains the downward target tracker node, whose job it is to detect and track targets seen in the downward fisheye camera. It also contains the line counter node (see [README_linecounter.md](README_linecounter.md)), because it needs the fisheye camera video and thus shares it for efficiency. These will run on the drone.

It also contains a debugger, that you can run on your computer, to verify that target tracking works correctly, and calibrate camera parameters, algorithm parameters, etc. It does not need to run during the competition, and is intended only for calibration and verification before the competition.

The tracker, when running correctly, will publish a list of tracked targets, [msg/tracks.msg](msg/tracks.msg), 60 times per second on the topic ```TRACKS_TOPIC```, which you can change in [src/parameters.h](src/parameters.h). Each target has a unique ID that you can use to refer to it. The ID persists while the target is tracked. Targets stop being tracked if they are not seen two seconds. The message can be used like this:

```
downward_target_tracker::tracks msg // From callback
int selected_id = 1234; // From planning algorithm, or manual selection
// ...
for (int i = 0; i < msg.num_targets)
{
    int id = msg.unique_id[i];
    float x = msg.position_x[i];
    float y = msg.position_y[i];
    float vx = msg.velocity_x[i];
    float vy = msg.velocity_y[i];

    if (id == selected_id)
    {
        // publish x,y,vx,vy to velocity-feedforward controller node?
    }
}
```

The debugger can be used to *select* a target by clicking on its list row in the GUI, and have its ```unique_id``` published on the topic ```SELECTED_TOPIC```, which you can change in [src/parameters.h](src/parameters.h).

### Compile

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

Clone [ascend_msgs](https://github.com/AscendNTNU/ascend_msgs) into catkin workspace.

### Test the tracker

**Step 1.** Set the parameters that you already know (like camera device name and topic names) in [src/parameters.h](src/parameters.h). Leave the rest alone for now.

**Step 2.** Run the tracker on the drone: ```$ rosrun downward_target_tracker tracker```.

**Step 3.** Ensure that your PC is on the same network as the drone and link your PC's ROS to the drone by typing in your PC terminal: ```$ export ROS_MASTER_URI=http://192.168.1.151:11311``` (IP is an example, replace with drone's IP. Port must be 11311).

**Step 4.** Run the debugger on your PC:  ```$ rosrun downward_target_tracker debugger```.

**Step 5.** Adjust camera controls to lighting: run [set_camera_controls.sh](/set_camera_controls.sh) script on the drone while tracker is running. It will set the camera exposure, gain, powerline frequency, etc., to values that are hardcoded in the script. Change the values in the script to get good image brightness, making sure that ```Frame rate``` stays at 60 Hz (see Timing window in debugger).

**Step 6.** Look at the "Default view" tab in the debugger. If you are:

* looking at a target (red or green plate)
* and you are either standing ONE METER above the ground at zero pitch and roll,
* or you are publishing the drone pose,

you should see the camera feed, a non-empty list of targets in a small GUI window, and bounding boxes around each in the image. Basically, something like this:

![](readme_img1.png)

If not, we need to calibrate intrinsics (fisheye parameters), extrinsics (camera mounting point), or color thresholds. If you need help, click the "Take a snapshot" button and send the files (snapshot*.jpg snapshot*.txt created in the directory you ran the debugger) to me.

**Calibrate color:** Click "Calibrate color" tab in debugger and follow guide that shows up.

**Calibrate camera:** Click "Calibrate camera" tab in debugger and follow guide that shows up.

**Save the parameters:** [src/parameters.h](src/parameters.h) contains the following parameters. If you change them, remember to **recompile**.

Parameter   | What
------------|-----
DEVICE_NAME         | i.e. /dev/video1
TESTING_WITH_LAPTOP | Just for me. Set this to 0.
DUMMY_IMAGE         | 0 will use USB camera; 1 will use a static image embedded in source code.
IMU_POSE_TOPIC      | Topic for best current estimate of drone pose (geometry_msgs::PoseStamped)
TRACKS_TOPIC        | Topic on which list of targets will be published (tracks.msg)
IMAGE_TOPIC         | Topic on which compressed camera feed will be published (image.msg)
INFO_TOPIC          | Topic on which debug info (adjustable parameters, bounding boxes and detections) will be published (info.msg)
LINE_COUNTER_TOPIC  | Topic on which [line counter output](README_linecounter.md) will be published
SELECTED_TOPIC      | Topic on which selected target ID will be published (std_msgs::Int32)
INFO_PUBLISH_INTERVAL | Change this to limit how often debug visualization info is sent (default is every frame)
IMAGE_PUBLISH_INTERVAL | Change this to limit how often compressed camera image is sent (default is every frame)
CAM_IMU_RX/Y/Z_INIT | Euler angles defining rotation from camera coordinates to imu coordinates
CAM_IMU_TX/Y/Z_INIT | Camera center relative imu center in imu coordinates
R_G/B/N_INIT | Color thresholds for 'red' classification (minimum red/green ratio, minimum red/blue ratio, and minimum average brightness)
G_R/B/N_INIT | Color thresholds for 'green' classification (minimum green/red ratio, minimum green/blue ratio, and minimum average brightness)
CAMERA_WIDTH/HEIGHT | Request video resolution from USB camera
CAMERA_BUFFERS | This might need to be changed if "Frame rate" (see Timing window in debugger) is less than 60 Hz
CAMERA_LEVELS | How many halvings of resolution should be done
CAMERA_F/U0/V0_INIT | Fisheye projection parameters, depends on video resolution
