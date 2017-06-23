### How do I compile you????

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

This will run the executable compiled in ```main_drone.cpp```.  A list of tracked targets is published 60 times per second in the topic ```downward_target_tracker/tracks```. See [```tracks.msg```](msg/tracks.msg). The message data can be used like this:

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

### Make sure things are working

Compile and run the tracker (on the drone).
```
$ rosrun downward_target_tracker tracker
```

Run the debugger (on your computer).
```
$ rosrun downward_target_tracker debugger
```

You will notice that a lot of stuff is weird. You will now fix that, and by doing so, learn how to:

* set publish rate limits
* calibrate the camera extrinsics and intrinsics
* calibrate color thresholds.

#### I'm getting a weird static image wtf?

Change ```#define DUMMY_IMAGE``` to ```0``` in ```main_drone.cpp```.

#### Failed to open device?

Change ```#define DEVICE_NAME "/dev/video42"``` to correct device name.

#### The debugger updates the image slowly...

Change ```INFO_PUBLISH_INTERVAL``` and ```IMAGE_PUBLISH_INTERVAL``` to 0 in ```main_drone.cpp```.

#### Ok... I'm not sure what's supposed to happen now?

If you are

* looking at a target (red or green plate)
* and you have the main tab open in the debugger,
* and you are either standing ONE METER above the ground,
* or you are publishing the drone's pose at the topic ```/mavros/vision_pose/pose```
* and you the pitch and roll are both zero in either case (we will get back to this)

then you should see a non-empty list of targets, and a bounding box around the target.

#### I'm doing that, but it's not seeing anything.

Either click the "Take a snapshot" button and send the files (snapshot*.jpg snapshot*.txt created in the directory you ran the debugger) to me. Or learn to calibrate the camera and color thresholds.

#### Calibrate color

Click the "calibrate color" tab in the debugger. Adjust the thresholds until satisfaction. Save the parameters by changing ```R_G_INIT, R_B_INIT, R_N_INIT, G_R_INIT, G_B_INIT, G_N_INIT``` in ```main_drone.cpp```.

#### Calibrate camera intrinsics

Click the "calibrate camera" tab in the debugger. Adjust ```f,u0,v0``` until lines look straight. Save the parameters by changing ```CAMERA_F_INIT, CAMERA_U0_INIT, CAMERA_V0_INIT``` in ```main_drone.cpp```.

#### Calibrate camera extrinsics

These define how the camera is rotated and translated relative to the IMU coordinate frame. By default they are aligned.

If the only difference between the coordinate frames is a rotation about the z-axis, you can calibrate it easily:
* Hold the drone parallel with the ground.
* Tilt the drone in the x or y axis.
* Adjust ```cam_imu_rz``` until the ground looks flat.

Assuming the drone pose is sent over mavros, OR that the drone is held at zero pitch and roll.

Save the parameters by changing ```CAM_IMU_RX_INIT, CAM_IMU_RY_INIT, CAM_IMU_RZ_INIT, CAM_IMU_TX_INIT, CAM_IMU_TY_INIT, CAM_IMU_TZ_INIT``` in ```main_drone.cpp```.
