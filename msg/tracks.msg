int32     num_targets    # Defines the number of elements in the arrays below.

int32[]   unique_id      # Each target has a unique ID that persists as long as
                         # the target is tracked.

float32[] position_x     # X and Y values are in the same coordinate frame as
float32[] position_y     # the drone's coordinate frame is relative to: i.e.
float32[] velocity_x     # the grid (with red line = x-axis). In other words
float32[] velocity_y     # position and velocity in the grid plane.

float32[] detection_rate # Detections per second divided by framerate. Flicker
                         # can cause a target to have a rate less than 1. If
                         # it is significantly lower than 1 the target is likely
                         # a false positive and should be ignored.

float32   time_until_180 # Seconds remaining until (assumingly) all targets will
                         # do the 180 degree turn. It is corrected live from any
                         # observed turns of tracked targets, and initialized to
                         # 20 seconds when the tracker is started with rosrun.
                         # (Start the tracker at the twenty second mark or otherwise
                         # do not trust this value until the boolean below is true).

bool      observed_180   # True if we have observed any target do a turn so far.
