#pragma once

#include <math.h>

// Projects a point (x,y,z) to pixel (u,v) and returns associated Jacobian evaluated
void im_project_equidistant(float f, float u0, float v0, float x, float y, float z, float *u, float *v)
//  f  (input): Equidistant fisheye camera model parameter (r = f x theta)
// u0  (input): Center of fisheye projection in x measured from left of image
// v0  (input): Center of fisheye projection in y measured from top of image
// xyz (input): Camera-space coordinate (OpenGL convention)
// uv (output): Pixel coordinate measured from top-left of image (DirectX convention)
{
    float l = sqrtf(x*x+y*y);
    if (l < 0.001f)
    {
        *u = u0;
        *v = v0;
    }
    else
    {
        float t = atanf(-l/z);
        float r = f*t;
        *u = u0 + r*x/l;
        *v = v0 - r*y/l;
    }
}

// Projects a point (x,y,z) to pixel (u,v) and returns associated Jacobian
void im_project_equidistant(
    float f, float u0, float v0,
    float x, float y, float z,
    float *u, float *v,
    float *dudx, float *dudy, float *dudz,
    float *dvdx, float *dvdy, float *dvdz)
{
    float l = sqrtf(x*x+y*y);
    float L = x*x + y*y + z*z;
    float t = atanf(-l/z);
    float r = f*t;
    if (l < 0.001f) // Handle the singularity in center
    {
        *u = u0;
        *v = v0;
        *dudx = 0.0f;
        *dudy = 0.0f;
        *dudz = x*f/L;
        *dvdx = 0.0f;
        *dvdy = 0.0f;
        *dvdz = -y*f/L;
    }
    else
    {
        float c = x/l;
        float s = y/l;
        float cc = c*c;
        float cs = c*s;
        float ss = s*s;
        float rl = r/l;
        float fL = f/L;
        *u = u0 + r*c;
        *v = v0 - r*s;
        *dudx =  ss*rl - cc*z*fL;
        *dudy = -cs*rl - cs*z*fL;
        *dudz =  x*fL;
        *dvdx =  cs*rl + cs*z*fL;
        *dvdy = -cc*rl + ss*z*fL;
        *dvdz = -y*fL;
    }
}

// Inverse-projects a pixel (u,v) to unit direction vector (x,y,z)
void im_ray_equidistant(float f, float u0, float v0, float u, float v, float *x, float *y, float *z)
//  f   (input): Equidistant fisheye camera model parameter (r = f x theta)
// u0   (input): Center of fisheye projection in x measured from left of image
// v0   (input): Center of fisheye projection in y measured from top of image
// uv   (input): Pixel coordinate measured from top-left of image (DirectX convention)
// xyz (output): Camera-space ray from camera origin through pixel (OpenGL convention)
{
    float du = u-u0;
    float dv = v0-v;
    float r = sqrtf(du*du+dv*dv);
    if (r > 1.0f)
    {
        float t = r / f;
        float s = sinf(t);
        float c = cosf(t);
        *x = s*du/r;
        *y = s*dv/r;
        *z = -c;
    }
    else
    {
        *x = 0.0f;
        *y = 0.0f;
        *z = -1.0f;
    }
}
