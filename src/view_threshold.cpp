void view_threshold(latest_image_t latest_image, downward_target_tracker::info latest_info)
{
    static unsigned char I_gray[CAMERA_WIDTH*CAMERA_HEIGHT];

    unsigned char *I_rgb = latest_image.I;
    int Ix = latest_image.Ix;
    int Iy = latest_image.Iy;
    float white_threshold_r = latest_info.white_threshold_r;
    float white_threshold_g = latest_info.white_threshold_g;
    float white_threshold_b = latest_info.white_threshold_b;
    float white_threshold_d = latest_info.white_threshold_d;

    asci_threshold(I_rgb, I_gray, Ix, Iy,
                   white_threshold_r,
                   white_threshold_g,
                   white_threshold_b,
                   white_threshold_d);

    vdbOrtho(-1.0f, +1.0f, +1.0f, -1.0f);
    vdbSetTexture2D(0, I_gray, Ix, Iy, GL_LUMINANCE, GL_UNSIGNED_BYTE);
    vdbDrawTexture2D(0);
}
