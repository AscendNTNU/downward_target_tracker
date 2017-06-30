// Basic global position tracking taken from state_estimation_node
// of last year, and position_fuser_node from this year. Has some
// bug fixes from position_fuser_node. Will merge into that repo
// later.

struct line_counter_filter_t
{
    // grid position and heading relative starting point
    double x;
    double y;
    double yaw;
    uint64_t t; // nanoseconds since UNIX epoch
};

struct line_counter_data_t
{
    // grid position modulo tile width
    // for the four possible heading relative x-axis estimates
    double x[4];
    double y[4];
    double yaw[4];
    uint64_t t; // nanoseconds since UNIX epoch
};

line_counter_filter_t line_counter_filter = {0};
ros::Publisher pub_vision_pose;

void line_counter_filter_set_state(double x, double y, double yaw)
{
    line_counter_filter.x = x;
    line_counter_filter.y = y;
    line_counter_filter.yaw = yaw;
    line_counter_filter.t = getnsec();
}

// Returns the equivalent angle in the range [-pi, pi]
double wrapAngle(double x)
{
    const double two_pi = 6.28318530718;
    double result = fmod(x, two_pi);
    if (result > +KF_PI) result -= two_pi;
    if (result < -KF_PI) result += two_pi;
    return result;
}

double floorNegativeInf(double x)
{
    return floor(x);
}

double angleDiff(double x1, double x2)
// Computes the smallest angle difference d, such that wrapAngle(x1) = wrapAngle(x2 + d)
{
    const double two_pi = 6.28318530718;
    double d1 = x1-x2;
    double d2 = x1+two_pi-x2;
    double d3 = x1-two_pi-x2;
    double d = d1;
    if (fabs(d2) < fabs(d)) d = d2;
    if (fabs(d3) < fabs(d)) d = d3;
    return d;
}

line_counter_filter_t
line_counter_filter_update(line_counter_filter_t prev, line_counter_data_t data)
{
    // putting these here for now
    const double MIN_LINECOUNTER_TIMESTEP = (1.0/100.0);
    const double LINECOUNTER_ACCEPT_LARGE_DELTA_AFTER_THIS_MUCH_TIME = (0.5);
    const double LINECOUNTER_X_SPEED_LIMIT = (7.0);
    const double LINECOUNTER_Y_SPEED_LIMIT = (7.0);
    const double LINECOUNTER_X_LPF_GAIN = (0.2);
    const double LINECOUNTER_Y_LPF_GAIN = (0.2);
    const double LINECOUNTER_YAW_LPF_GAIN = (0.8);
    const double TILE_WIDTH = 1.0;
    const double LINECOUNTER_JUMP_THRESHOLD = (0.5);

    double dt = (double)(data.t - prev.t) / 1e9;
    if (dt < 0.0)
    {
        // @ Properly handle timestamps
        dt += 1.0;
    }
    if (dt < MIN_LINECOUNTER_TIMESTEP)
    {
        KF_LOG("[t=%.2f] dt (%.2f) < MIN_LINECOUNTER_TIMESTEP (%.2f)\n", data.t, dt, MIN_LINECOUNTER_TIMESTEP);
        dt = MIN_LINECOUNTER_TIMESTEP;
    }

    double px0  = prev.x; // Previous absolute x coordinate
    double py0  = prev.y; // Previous absolute y coordinate
    double yaw0 = prev.yaw; // Previously estimated yaw [-pi, pi]

    // Find the solution with the closest yaw angle and
    // extract the tile-relative coordinate for that angle.
    double yaw1;
    double tile_x1;
    double tile_y1;
    {
        yaw1 = data.yaw[0];
        tile_x1 = data.x[0];
        tile_y1 = data.y[0];
        for (int i = 0; i < 4; i++)
        {
            double this_diff = fabs(angleDiff(data.yaw[i], yaw0));
            double curr_diff = fabs(angleDiff(yaw1, yaw0));
            bool this_is_smaller = this_diff < curr_diff;
            if (this_is_smaller)
            {
                yaw1 = data.yaw[i];
                tile_x1 = data.x[i];
                tile_y1 = data.y[i];
            }
        }
    }

    // Compute the closest absolute position based on the previously
    // estimated absolute position, and the newly measured tile-relative
    // position.

    double discrete_x = floorNegativeInf(px0 / TILE_WIDTH);
    double discrete_y = floorNegativeInf(py0 / TILE_WIDTH);
    double px1 = discrete_x + tile_x1;
    double py1 = discrete_y + tile_y1;

    if (px1-px0 > LINECOUNTER_JUMP_THRESHOLD)
        px1 -= TILE_WIDTH;
    if (px1-px0 < -LINECOUNTER_JUMP_THRESHOLD)
        px1 += TILE_WIDTH;

    if (py1-py0 > LINECOUNTER_JUMP_THRESHOLD)
        py1 -= TILE_WIDTH;
    if (py1-py0 < -LINECOUNTER_JUMP_THRESHOLD)
        py1 += TILE_WIDTH;

    // This ensures that yaw0 can be lowpass filtered toward
    // yaw1 in the direction of least angular difference, and
    // also implies that yaw1 is not necessarily in the range
    // [-pi, pi] after filtering. But we will wrap it to the
    // correct range after the lowpass filter.
    double delta_yaw = angleDiff(yaw1, yaw0);

    // Lowpass filter incoming data with an exponential moving average filter
    double  px1_lpf =  px0 + LINECOUNTER_X_LPF_GAIN * (px1 - px0);
    double  py1_lpf =  py0 + LINECOUNTER_Y_LPF_GAIN * (py1 - py0);
    double yaw1_lpf = yaw0 + LINECOUNTER_YAW_LPF_GAIN * delta_yaw;

    yaw1_lpf = wrapAngle(yaw1_lpf);

    // Estimate velocity in x and y position
    double dx = (px1_lpf-px0) / dt;
    double dy = (py1_lpf-py0) / dt;

    line_counter_filter_t curr = prev;

    if (fabs(dx) <= LINECOUNTER_X_SPEED_LIMIT &&
        fabs(dy) <= LINECOUNTER_Y_SPEED_LIMIT)
    // Velocity was small enough
    {
        curr.x = px1_lpf;
        curr.y = py1_lpf;
        curr.yaw = yaw1_lpf;
        curr.t = data.t;
    }
    else if (dt >= LINECOUNTER_ACCEPT_LARGE_DELTA_AFTER_THIS_MUCH_TIME)
    // Velocity was too big
    // But sufficiently enough time has passed since we accepted a measurement
    {
        curr.x = px1_lpf;
        curr.y = py1_lpf;
        curr.yaw = yaw1_lpf;
        curr.t = data.t;
    }
    else
    // Velocity was too big and not much time has passed
    // It's probably just noise! So let's ignore it.
    {
        // Intentionally do not update ANY fields of curr
    }

    return curr;
}

void filter_and_publish_vision_pose(ascend_msgs::LineCounter msg)
{
    // PACK DATA
    line_counter_data_t data;
    {
        data.t = msg.timestamp;
        data.x[0] = msg.x1;
        data.x[1] = msg.x2;
        data.x[2] = msg.x3;
        data.x[3] = msg.x4;
        data.y[0] = msg.y1;
        data.y[1] = msg.y2;
        data.y[2] = msg.y3;
        data.y[3] = msg.y4;
        data.yaw[0] = msg.yaw1;
        data.yaw[1] = msg.yaw2;
        data.yaw[2] = msg.yaw3;
        data.yaw[3] = msg.yaw4;
    }

    // UPDATE FILTER WITH LATEST MEASUREMENT
    line_counter_filter = line_counter_filter_update(line_counter_filter, data);

    // PUBLISH LATEST FILTER ESTIMATES
    {
        geometry_msgs::PoseStamped msg;
        double yaw = line_counter_filter.yaw;
        double x = line_counter_filter.x;
        double y = line_counter_filter.y;
        double t = line_counter_filter.t;

        // construct quaternion from zero pitch/roll, and only yaw
        // (not sure if this is OK or not!!)
        msg.pose.orientation.x = 0.0f;
        msg.pose.orientation.y = 0.0f;
        msg.pose.orientation.z = sinf(yaw/2.0f);
        msg.pose.orientation.w = cosf(yaw/2.0f);

        msg.pose.position.x = x;
        msg.pose.position.y = y;

        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "world"; // for Rviz?

        pub_vision_pose.publish(msg);
    }
}
