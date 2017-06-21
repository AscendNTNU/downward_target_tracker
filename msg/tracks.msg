int32     num_targets    # Defines the number of elements in the arrays below

int32[]   unique_id      # Each target has a unique ID that persist as long as
                         # the target is tracked.

float32[] position_x     # X and Y values are in the same coordinate frame as
float32[] position_y     # the drone's coordinate frame is relative to: i.e.
float32[] velocity_x     # the grid (with red line = x-axis). In other words
float32[] velocity_y     # position and velocity in the grid plane.

float32[] detection_rate # (normalized) detections per second
                         # A target can at most have 60 detections per second
                         # if the camera is running at 60 fps. Hence, a target
                         # which is detected 30 times per second will have a
                         # rate of 0.5.
