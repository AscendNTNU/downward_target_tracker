// asc_grid_detector.h - ver 13
//
// SIMD-optimized line detection and room-scale grid estimation in fisheye
// and pinhole cameras.
//
// Written by: Simen Haugo (simen.haugo@ascendntnu.no)
// For:        Ascend NTNU (ascendntnu.no)
//
// Changelog
// ------------------------------------------------------------------------
// ver 13: Tested to work at idrettshall with brown paper. (Tuned LPF gain).
// ver 12: Various sanity checks on the math.
// ver 11: Height adaptation actually was correct (math-wise), so removed
//         the todo.
// ver 10: asci_angle: Does not use a while loop to wrap the input angle.

// ver 9: Experimental support for low altitudes where a full tile might not
//        be visible.
// ver 8: The low pass filter now associates at most one new maxima to match
//        a given tile maxima of the previous frame, instead of associating
//        all maximas that are sufficiently close.
// ver 7: Low pass filter with previous frame's tile estimate
//        (Doesn't help if we detect different tiles in subsequent frames)
//        (Can be enabled by #define'ing ASCI_GRID_ENABLE_LPF)
//        (Parameters are available as #defines, see below.)
// ver 6: Taking tile width as an option parameter
// ver 5: Using new projection and unprojection routines, formalized
//        coordinate transforms.
// ver 4: Improved accuracy and positive detection rate in clutter.
// ver 3: Increased weighting on perpendicularity of the selected tile.
// ver 2: Can estimate height based on observed grid size.
// ver 1: Initial version.
//
// How to compile:
// ------------------------------------------------------------------------
// This file contains both the header file and the implementation file.
// To compile, insert the following in A SINGLE source file in your project
//
//     #define ASC_GRID_DETECTOR_IMPLEMENTATION
//     #include "asc_line_detector.h"
//
// You may otherwise include this file as you would include a traditional
// header file. You may choose between SSE, AVX or no optimizations by
// inserting the following in the same source file as above
//
//     #define ASC_GRID_DETECTOR_AVX // Enable AVX/AVX2 level optimization
//     #define ASC_GRID_DETECTOR_SSE // Enable SSE/SSE2 level optimization
//
// Default is no optimization.
//
// You can avoid including <assert.h> by defining your own version
// of ASCI_ASSERT.

#ifndef ASC_GRID_DETECTOR_HEADER_INCLUDE
#define ASC_GRID_DETECTOR_HEADER_INCLUDE
#define ASCI_MAX_WIDTH (1920)
#define ASCI_MAX_HEIGHT (1080)
#define ASCI_GRID_PREDICTED_LINE_COST (0.16f)
#include <stdint.h>
typedef int64_t  s64;
typedef int32_t  s32;
typedef int16_t  s16;
typedef int8_t   s08;
typedef uint64_t u64;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t  u08;
typedef float    r32;

struct asc_GridOptions
{
    s16 sobel_threshold;
    s32 maxima_threshold;
    r32 max_error;

    r32 tile_width; // The length of the square tiles of the grid [meters]

    // The grid detector supports input images following the equidistant fisheye model.

    bool correct_fisheye;
    r32 fisheye_f;        // Fisheye lens parameter in the equidistant camera model (r = f theta)
    r32 fisheye_center_x; // Center of distortion in x (measured from left in image) [pixels]
    r32 fisheye_center_y; // Center of distortion in y (measured from top in image) [pixels]

    // If correct_fisheye is enabled, the grid detector will produce a pinhole-equivalent
    // projection of the input, with these parameters. If it is disabled, i.e. the input
    // is from a pinhole-model camera, these parameters must match those of your camera.

    r32 pinhole_fov_x;    // Horizontal field of view of the pinhole projection
    r32 pinhole_center_x; // Image center in x (measured from left in image) [pixels]
    r32 pinhole_center_y; // Image center in y (measured from top in image) [pixels]

    // The grid detector can compare the observed tile size with the predicted
    // size at the given input height, and refine the height estimate by a
    // simple feedback law. The adapation rate determines how quickly it
    // reacts. 1 implies immediate adaptation, while 0 implies no adaptation
    // (keep input height).
    r32 height_adaptation_rate;
};

// If the detector sees a tile, there are four possible poses that the drone can have,
// since the tile looks identical when rotated multiples of PI/2. The detector returns
// all of these.
struct asc_GridResult
{
    r32 yaw[4]; // The angle between the camera x-axis and the world x-axis, i.e. R_c^w = R_z(yaw)
                // wrapped to [-pi, pi]

    r32 x[4];   // The location of the camera in the world frame, modulo tile width [0, tile width),
    r32 y[4];   // relative to the lower-left corner of an observed tile when derotated by yaw[i].

    r32 error;  // A measure of the quality of the estimate based on the squareness of the observed
                // tile, and its size. Good estimates have an error << 1.

    r32 height; // Estimated height of camera
};

asc_GridResult asc_find_grid(u08 *in_gray, s32 in_width, s32 in_height,
                             r32 camera_ex, r32 camera_ey, r32 camera_z,
                             asc_GridOptions options,
                             u08 *in_rgb = 0);

#endif

#ifdef ASC_GRID_DETECTOR_IMPLEMENTATION
#define SO_MATH_NO_PI
#include "so_math.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#ifndef ASCI_PI
#define ASCI_PI 3.1415926f
#endif

#ifndef ASCI_TWO_PI
#define ASCI_TWO_PI 6.28318530718f
#endif

#ifndef ASCI_ASSERT
#include <assert.h>
#define ASCI_ASSERT assert
#endif

#define countof(x) (sizeof((x))/sizeof((x)[0]))
#define forx(START, END) for (s32 x = START; x < END; x++)
#define fory(START, END) for (s32 y = START; y < END; y++)
#define fori(START, END) for (s32 i = START; i < END; i++)
#define forxo(END) for (s32 x = 0; x < END; x++)
#define foryo(END) for (s32 y = 0; y < END; y++)
#define forio(END) for (s32 i = 0; i < END; i++)
#define forjo(END) for (s32 j = 0; j < END; j++)

s32 asci_clamp_s32(s32 x, s32 low, s32 high)
{
    if (x < low) return low;
    if (x > high) return high;
    return x;
}

s32 asci_round_positive(r32 x) { return (s32)(x+0.5f); }
s32 asci_floor_positive(r32 x) { return (s32)(x); }
r32 asci_max(r32 x, r32 y) { if (x > y) return x; else return y; }
s32 asci_min_s32(s32 x, s32 y) { if (x < y) return x; else return y; }
s32 asci_max_s32(s32 x, s32 y) { if (x > y) return x; else return y; }
s16 asci_abs_s16(s16 x) { return abs(x); }
r32 asci_abs_r32(r32 x) { return fabs(x); }
r32 asci_square(r32 x) { return x * x; }

// Wraps an angle x between [-pi, pi]
r32 asci_angle(r32 x)
{
    x = fmod(x, ASCI_TWO_PI);
    if (x > ASCI_PI)
    {
        x -= ASCI_TWO_PI;
    }
    if (x < -ASCI_PI)
    {
        x += ASCI_TWO_PI;
    }
    return x;
}

r32 asci_modulo(r32 x, r32 m)
{
    // Implements:
    // while (x < 0.0f) x += m;
    // while (x > m)    x -= m;
    if (x < 0.0f)
    {
        s32 n = asci_floor_positive(-x / m);
        r32 r = m - (-x - n*m);
        return r;
    }
    else
    {
        s32 n = asci_floor_positive(x / m);
        r32 r = x - n*m;
        return r;
    }
}

// This algorithm has a maximal period of 2^128 − 1.
// https://en.wikipedia.org/wiki/Xorshift
u32 asci_xor128()
{
    static u32 x = 123456789;
    static u32 y = 362436069;
    static u32 z = 521288629;
    static u32 w = 88675123;
    u32 t = x ^ (x << 11);
    x = y;
    y = z;
    z = w;
    w = w ^ (w >> 19) ^ t ^ (t >> 8);
    return w;
}

void asci_threshold(u08 *in_rgb, u08 *out_gray, s32 w, s32 h, r32 rt, r32 gt, r32 bt, r32 dt)
{
    for (int y = 0; y < h; y++)
    for (int x = 0; x < w; x++)
    {
        u08 *pixel = &in_rgb[(y*w+x)*3];
        r32 r = (r32)pixel[0];
        r32 g = (r32)pixel[1];
        r32 b = (r32)pixel[2];

        r32 dr = fabsf(r - rt);
        r32 dg = fabsf(g - gt);
        r32 db = fabsf(b - bt);
        r32 dd = (dr + dg + db) / 3.0f;

        u08 result;
        if (dd < dt)
        {
            r32 result_real = (r + r + b + g + g + g) / 6.0f;
            result_real *= 1.0f - dd/dt;
            s32 result_rounded = (s32)result_real;
            if (result_rounded < 0) result_rounded = 0;
            if (result_rounded > 255) result_rounded = 255;
            result = (u08)result_rounded;
        }
        else
        {
            result = 0;
        }
        out_gray[y*w+x] = result;
    }
}

struct asci_Feature
{
    s32 x;
    s32 y;
    s16 gx;
    s16 gy;
    s16 gg;
};

#ifdef ASC_GRID_DETECTOR_SSE
#include "xmmintrin.h"
#include "emmintrin.h"
#include "smmintrin.h"
void asci_sobel(
    u08 *in_gray,
    s32  in_width,
    s32  in_height,
    s16  threshold,
    asci_Feature *out_features,
    s32 *out_feature_count)
{
    s32 count = 0;
    for (s32 y = 1; y < in_height-1; y++)
    {
        for (s32 x = 1; x <= in_width-16; x += 16)
        {
            u08 *addr00 = in_gray + (y-1)*in_width + x-1;
            u08 *addr01 = in_gray + (y-1)*in_width + x;
            u08 *addr02 = in_gray + (y-1)*in_width + x+1;

            u08 *addr10 = in_gray + (y)*in_width + x-1;
            u08 *addr12 = in_gray + (y)*in_width + x+1;

            u08 *addr20 = in_gray + (y+1)*in_width + x-1;
            u08 *addr21 = in_gray + (y+1)*in_width + x;
            u08 *addr22 = in_gray + (y+1)*in_width + x+1;

            __m128i source00 = _mm_loadu_si128((__m128i*)addr00);
            __m128i source01 = _mm_loadu_si128((__m128i*)addr01);
            __m128i source02 = _mm_loadu_si128((__m128i*)addr02);

            __m128i source10 = _mm_loadu_si128((__m128i*)addr10);
            __m128i source12 = _mm_loadu_si128((__m128i*)addr12);

            __m128i source20 = _mm_loadu_si128((__m128i*)addr20);
            __m128i source21 = _mm_loadu_si128((__m128i*)addr21);
            __m128i source22 = _mm_loadu_si128((__m128i*)addr22);

            // divide pixels by 4

            __m128i shift_mask = _mm_set1_epi8(0x3F);
            source00 = _mm_and_si128(shift_mask, _mm_srli_epi16(source00, 2));
            source01 = _mm_and_si128(shift_mask, _mm_srli_epi16(source01, 2));
            source02 = _mm_and_si128(shift_mask, _mm_srli_epi16(source02, 2));

            source10 = _mm_and_si128(shift_mask, _mm_srli_epi16(source10, 2));
            source12 = _mm_and_si128(shift_mask, _mm_srli_epi16(source12, 2));

            source20 = _mm_and_si128(shift_mask, _mm_srli_epi16(source20, 2));
            source21 = _mm_and_si128(shift_mask, _mm_srli_epi16(source21, 2));
            source22 = _mm_and_si128(shift_mask, _mm_srli_epi16(source22, 2));

            // I compute the x and y gradients in their positive
            // and negative components, to fit everything in u08
            // values.

            // TODO: Div only by two for source12, source10,
            // source21 and source01.

            // px
            __m128i positive_x = _mm_set1_epi8(0);
            positive_x = _mm_adds_epu8(positive_x, source02);
            positive_x = _mm_adds_epu8(positive_x, source12);
            positive_x = _mm_adds_epu8(positive_x, source12);
            positive_x = _mm_adds_epu8(positive_x, source22);

            // nx
            __m128i negative_x = _mm_set1_epi8(0);
            negative_x = _mm_adds_epu8(negative_x, source00);
            negative_x = _mm_adds_epu8(negative_x, source10);
            negative_x = _mm_adds_epu8(negative_x, source10);
            negative_x = _mm_adds_epu8(negative_x, source20);

            // py
            __m128i positive_y = _mm_set1_epi8(0);
            positive_y = _mm_adds_epu8(positive_y, source20);
            positive_y = _mm_adds_epu8(positive_y, source21);
            positive_y = _mm_adds_epu8(positive_y, source21);
            positive_y = _mm_adds_epu8(positive_y, source22);

            // ny
            __m128i negative_y = _mm_set1_epi8(0);
            negative_y = _mm_adds_epu8(negative_y, source00);
            negative_y = _mm_adds_epu8(negative_y, source01);
            negative_y = _mm_adds_epu8(negative_y, source01);
            negative_y = _mm_adds_epu8(negative_y, source02);

            // Approximate magnitude of gradient by absolute value

            // x
            __m128i abs_gx = _mm_subs_epu8(
                _mm_max_epu8(positive_x, negative_x),
                _mm_min_epu8(positive_x, negative_x));

            // y
            __m128i abs_gy = _mm_subs_epu16(
                _mm_max_epu8(positive_y, negative_y),
                _mm_min_epu8(positive_y, negative_y));

            __m128i magnitude = _mm_adds_epu8(abs_gx, abs_gy);

            __m128i skip_value = _mm_set1_epi8(threshold);
            __m128i skip_cmp = _mm_cmplt_epi8(magnitude, skip_value);
            int move_mask = _mm_movemask_epi8(skip_cmp);
            if (move_mask == 0xffff)
            {
                continue;
            }

            u08 dst_magnitude[16];
            u08 dst_positive_x[16];
            u08 dst_negative_x[16];
            u08 dst_positive_y[16];
            u08 dst_negative_y[16];
            _mm_storeu_si128((__m128i*)dst_magnitude,  magnitude);
            _mm_storeu_si128((__m128i*)dst_positive_x, positive_x);
            _mm_storeu_si128((__m128i*)dst_negative_x, negative_x);
            _mm_storeu_si128((__m128i*)dst_positive_y, positive_y);
            _mm_storeu_si128((__m128i*)dst_negative_y, negative_y);

            // @ SIMD Sobel

            for (s32 dx = 0; dx < 16; dx++)
            {
                if (dst_magnitude[dx] > threshold)
                {
                    asci_Feature feature = {0};
                    feature.x = x+dx;
                    feature.y = y;
                    feature.gx = (s16)dst_positive_x[dx]-(s16)dst_negative_x[dx];
                    feature.gy = (s16)dst_positive_y[dx]-(s16)dst_negative_y[dx];
                    feature.gg = (s16)dst_magnitude[dx];
                    out_features[count++] = feature;
                }
            }
        }
    }
    *out_feature_count = count;
}
#else
void asci_sobel(
    u08 *in_gray,
    s32  in_width,
    s32  in_height,
    s16  threshold,
    asci_Feature *out_features,
    s32 *out_feature_count)
{
    s32 feature_count = 0;
    for (s32 y = 1; y < in_height-1; y++)
    {
        for (s32 x = 1; x < in_width-1; x++)
        {
            s16 i00 = (s16)in_gray[(y-1)*in_width+x-1] >> 2;
            s16 i01 = (s16)in_gray[(y-1)*in_width+x] >> 2;
            s16 i02 = (s16)in_gray[(y-1)*in_width+x+1] >> 2;
            s16 i20 = (s16)in_gray[(y+1)*in_width+x-1] >> 2;
            s16 i21 = (s16)in_gray[(y+1)*in_width+x] >> 2;
            s16 i22 = (s16)in_gray[(y+1)*in_width+x+1] >> 2;
            s16 i10 = (s16)in_gray[y*in_width+x-1] >> 2;
            s16 i12 = (s16)in_gray[y*in_width+x+1] >> 2;

            s16 gx = i02-i00+i12+i12-i10-i10+i22-i20;
            s16 gy = i20-i00+i21+i21-i01-i01+i22-i02;
            s16 gg = asci_abs_s16(gx) + asci_abs_s16(gy);
            if (gg > threshold)
            {
                asci_Feature feature = {0};
                feature.x = x;
                feature.y = y;
                feature.gx = gx;
                feature.gy = gy;
                feature.gg = gg;
                out_features[feature_count++] = feature;
            }
        }
    }
    *out_feature_count = feature_count;
}
#endif

void asci_fisheye_undistort(
    asci_Feature *in_features,
    s32 in_count,
    s32 in_width,
    s32 in_height,
    asci_Feature *out_features,
    s32 *out_count,
    r32 fisheye_f,
    r32 fisheye_center_x,
    r32 fisheye_center_y,
    r32 pinhole_fov_x)
{
    r32 ff = fisheye_f;
    r32 fp = (in_width/2.0f) / tan(pinhole_fov_x/2.0f);
    r32 clip_theta = pinhole_fov_x/2.0f;

    s32 count = 0;
    for (s32 i = 0; i < in_count; i++)
    {
        asci_Feature feature = in_features[i];
        r32 xd = feature.x - fisheye_center_x;
        r32 yd = feature.y - fisheye_center_y;
        r32 rd = sqrt(xd*xd+yd*yd);
        r32 theta = rd/ff;

        if (theta > clip_theta)
            continue;

        if (rd < 1.0f) // Skip pixels in center to avoid dividing by zero later
            continue;

        r32 ru = fp*tan(theta);

        r32 xu = (xd/rd)*ru; // Safe divide: rd >= 1.0f
        r32 yu = (yd/rd)*ru; // Safe divide: rd >= 1.0f

        r32 gxu = 0.0f;
        r32 gyu = 0.0f;
        {
            r32 gxd = (r32)feature.gx;
            r32 gyd = (r32)feature.gy;
            r32 DIDphid = -gxd*yd + gyd*xd;
            r32 DIDrd = (gxd*xd+gyd*yd)/rd; // Safe divide: rd >= 1.0f

            r32 DrdDru = (ff*fp)/(ru*ru+fp*fp);
            r32 DIDphiu = DIDphid;
            r32 DIDru = DIDrd*DrdDru;

            r32 DruDxu = xu/ru; // Safe divide: ru >= fp tan(1.0f / ff)
            r32 DruDyu = yu/ru; // Safe divide: ru >= fp tan(1.0f / ff)

            r32 DphiuDxu = 0.0f;
            r32 DphiuDyu = 0.0f;
            if (asci_abs_r32(yu) > 1.0f)
                DphiuDxu = ((xu*xu)/(ru*ru)-1.0f)/yu;
            if (asci_abs_r32(xu) > 1.0f)
                DphiuDyu = (1.0f-(yu*yu)/(ru*ru))/xu;

            gxu = DIDru*DruDxu + DIDphiu*DphiuDxu;
            gyu = DIDru*DruDyu + DIDphiu*DphiuDyu;

            // Renormalize
            r32 ggu = sqrt(gxu*gxu+gyu*gyu);
            gxu *= feature.gg/ggu; // Probably safe divide...?
            gyu *= feature.gg/ggu; // Probably safe divide...?
        }

        s32 ix = asci_round_positive(fisheye_center_x+xu);
        s32 iy = asci_round_positive(fisheye_center_y+yu);

        feature.x = ix;
        feature.y = iy;
        feature.gx = (s16)gxu;
        feature.gy = (s16)gyu;
        out_features[count++] = feature;
    }

    *out_count = count;
}

s32 _asci_tr_to_i(s32 bins_t, s32 bins_r, s32 ti, s32 ri)
{
    ASCI_ASSERT(ti >= 0 && ti < bins_t &&
                ri >= 0 && ri < bins_r);
    return ti + ri*bins_t;
}

#define tr_to_i(ti, ri) _asci_tr_to_i(bins_t, bins_r, ti, ri)

struct asci_HoughCell
{
    r32 avg_r;
    r32 avg_t;
    s32 count;
};

int asci_cmp_rt(const void *p1, const void *p2)
{
    asci_HoughCell c1 = *(asci_HoughCell*)p1;
    asci_HoughCell c2 = *(asci_HoughCell*)p2;
    if      (c1.count < c2.count) return +1;
    else if (c1.count > c2.count) return -1;
    else                          return 0;
}

void asci_hough_transform(
    asci_HoughCell *histogram,
    s32 bins_t,
    s32 bins_r,
    r32 t_min,
    r32 t_max,
    r32 r_min,
    r32 r_max,
    asci_Feature *features,
    s32 feature_count,
    s32 *out_peak_count)
{
    for (s32 i = 0; i < bins_t*bins_r; i++)
    {
        histogram[i].count = 0;
        histogram[i].avg_t = 0.0f;
        histogram[i].avg_r = 0.0f;
    }

    s32 peak_count = 0;
    for (s32 i = 0; i < feature_count; i++)
    {
        asci_Feature f = features[i];
        r32 x = f.x;
        r32 y = f.y;

        r32 t_0 = atan2((r32)f.gy, (r32)f.gx);
        if (t_0 < 0.0f)
            t_0 += ASCI_PI;

        s32 ti_0 = asci_floor_positive(bins_t * t_0 / ASCI_PI);
        s32 ti_delta = 1;

        for (s32 ti_unwrapped = ti_0-ti_delta; ti_unwrapped <= ti_0+ti_delta; ti_unwrapped++)
        {
            s32 ti = ti_unwrapped;
            if (ti < 0) ti += bins_t;
            if (ti > bins_t-1) ti -= bins_t;

            r32 t = ASCI_PI * ti / (r32)bins_t;
            r32 r = x*cos(t)+y*sin(t);
            s32 ri = asci_floor_positive(bins_r * (r - r_min) / (r_max - r_min));
            ri = asci_clamp_s32(ri, 0, bins_r-1);

            asci_HoughCell *hc = &histogram[tr_to_i(ti, ri)];
            hc->avg_t += t;
            hc->avg_r += r;
            s32 new_count = ++hc->count;
            if (new_count > peak_count)
                peak_count = new_count;
        }

        // This is an optimization. In some cases, it is sufficient
        // to only compute the Hough transform of a subset of the
        // input features. Since a uniform skip length might systematically
        // ignore good features, I skip a variable amount each iteration.
        // i += (asci_xor128() % 2);
    }

    // Compute cell averages
    for (s32 i = 0; i < bins_t*bins_r; i++)
    {
        if (histogram[i].count > 0)
        {
            histogram[i].avg_t /= (r32)histogram[i].count;
            histogram[i].avg_r /= (r32)histogram[i].count;
        }
    }

    *out_peak_count = peak_count;
}

void asci_dilate_histogram(
    asci_HoughCell *histogram,
    s32 *dilated_counts,
    s32 bins_t,
    s32 bins_r,
    s32 radius_t,
    s32 radius_r)
{
    for (s32 i = 0; i < bins_t*bins_r; i++)
    {
        dilated_counts[i] = 0;
    }

    for (s32 ri_center = 0; ri_center < bins_r; ri_center++)
    for (s32 ti_center = 0; ti_center < bins_t; ti_center++)
    {
        s32 c_max = histogram[tr_to_i(ti_center, ri_center)].count;

        // This is an optimization. I don't think it has a large
        // effect (if any?) on the result, but it gives a huge
        // speed up, since the loop skips over areas of low activity.
        if (c_max < 30)
            continue;

        s32 ri0 = asci_max_s32(ri_center-radius_r, 0);
        s32 ri1 = asci_min_s32(ri_center+radius_r, bins_r-1);
        s32 ti0 = ti_center-radius_t;
        s32 ti1 = ti_center+radius_t;
        for (s32 ri = ri0; ri <= ri1; ri++)
        for (s32 ti = ti0; ti <= ti1; ti++)
        {
            // Identification with theta = 0 and theta = pi
            s32 ti_wrapped = ti;
            s32 ri_wrapped = ri;
            if (ti < 0)
            {
                ti_wrapped = bins_t+ti;
                ri_wrapped = bins_r-1-ri;
            }
            else if (ti > bins_t-1)
            {
                ti_wrapped = ti-bins_t;
                ri_wrapped = bins_r-1-ri;
            }

            s32 c = histogram[tr_to_i(ti_wrapped, ri_wrapped)].count;
            c_max = asci_max_s32(c, c_max);
        }
        dilated_counts[tr_to_i(ti_center, ri_center)] = c_max;
    }
}

void asci_find_local_maxima(
    asci_HoughCell *histogram,
    s32 *dilated_counts,
    s32 bins_t,
    s32 bins_r,
    s32 width_t,
    s32 width_r,
    s32 threshold,
    s32 max_count,
    asci_HoughCell *out_list,
    s32 *out_count)
{
    s32 count = 0;

    // Extract maxima
    for (s32 ri_center = 0; ri_center < bins_r; ri_center++)
    for (s32 ti_center = 0; ti_center < bins_t; ti_center++)
    {
        s32 i_center = tr_to_i(ti_center, ri_center);

        // Compare against absolute minimum threshold
        s32 c_original = histogram[i_center].count;
        if (c_original < 1) continue;

        // Compare against neighborhood maxima
        s32 c_dilated = dilated_counts[i_center];
        if (c_original != c_dilated) continue;

        r32 avg_t = histogram[i_center].avg_t;
        r32 avg_r = histogram[i_center].avg_r;
        s32 sum_n = c_original;

        if (sum_n > threshold)
        {
            asci_HoughCell h = {0};
            h.avg_t = avg_t;
            h.avg_r = avg_r;
            h.count = sum_n;
            out_list[count++] = h;
        }
    }
    *out_count = count;
}

typedef vec2 pixel2f;
typedef vec3 camera3f;
typedef vec2 world2f;

camera3f
asci_pinhole_camera_ray(r32 f, r32 u0, r32 v0, pixel2f uv)
//  f (input): Focal length of pinhole camera model
// u0 (input): Center of pinhole projection in x measured from left of image
// v0 (input): Center of pinhole projection in y measured from top of image
// uv (input): Pixel coordinate measured from top-left of image (DirectX convention)
//     return: Normalized camera-space ray passing through pixel (OpenGL convention)
{
    r32 du =   uv.x-u0;
    r32 dv = -(uv.y-v0);
    r32 invlen = 1.0f / sqrt(du*du + dv*dv + f*f);
    r32 rx =  du * invlen;
    r32 ry =  dv * invlen;
    r32 rz = -f * invlen;
    vec3 result = { rx, ry, rz };
    return result;
}

world2f
asci_trace_world_plane(r32 f, r32 u0, r32 v0, mat3 R, r32 h, pixel2f uv)
//  f (input): Focal length of pinhole camera model
// u0 (input): Center of pinhole projection in x measured from left of image
// v0 (input): Center of pinhole projection in y measured from top of image
//  R (input): R^w_c camera frame expressed in world frame (OpenGL convention)
//  h (input): The camera frame's height above world plane
// uv (input): Pixel coordinate measured from top-left of image (DirectX convention)
//     return: World-space X and Y coordinate of intersection point between
//             world plane and the world-space ray passing through uv.
{
    vec3 rc = asci_pinhole_camera_ray(f, u0, v0, uv);
    vec3 rw = R*rc;
    r32 t = -h / rw.z;
    r32 X = t*rw.x;
    r32 Y = t*rw.y;
    vec2 result = { X, Y };
    return result;
}

pixel2f
asci_project_fisheye(r32 f, r32 u0, r32 v0, camera3f p)
//  f (input): Fisheye camera model parameter
// u0 (input): Center of fisheye projection in x measured from left of image
// v0 (input): Center of fisheye projection in y measured from top of image
//  p (input): Camera-space coordinate (OpenGL convention)
//     return: Image-space coordinate of projected point (DirectX convention)
{
    r32 l = sqrt(p.x*p.x+p.y*p.y);
    r32 t = atan(-l/p.z);
    r32 r = f*t;
    r32 du = r*p.x/l;
    r32 dv = r*p.y/l;
    r32 u = u0 + du;
    r32 v = v0 - dv;
    vec2 result = { u, v };
    return result;
}

pixel2f
asci_project_pinhole(r32 f, r32 u0, r32 v0, camera3f p)
//  f (input): Focal length of pinhole camera model
// u0 (input): Center of pinhole projection in x measured from left of image
// v0 (input): Center of pinhole projection in y measured from top of image
//  p (input): Camera-space coordinate (OpenGL convention)
//     return: Image-space coordinate of projected point (DirectX convention)
{
    r32 invnegz = -1.0f / p.z;
    r32 du = f*p.x*invnegz;
    r32 dv = f*p.y*invnegz;
    r32 u = u0 + du;
    r32 v = v0 - dv;
    vec2 result = { u, v };
    return result;
}

void
asci_line_to_segment(r32 normal_x, r32 normal_y, r32 r, r32 x_min, r32 x_max, r32 y_min, r32 y_max, r32 *out_x1, r32 *out_y1, r32 *out_x2, r32 *out_y2)
// Computes the line endpoints from the (normal, distance) parametrization
{
    r32 x1, y1, x2, y2;
    if (fabs(normal_y) > fabs(normal_x))
    {
        x1 = x_min;
        y1 = (r-x1*normal_x)/normal_y;

        if (y1 < y_min)
        {
            y1 = y_min;
            x1 = (r-y1*normal_y)/normal_x;
        }
        else if (y1 > y_max)
        {
            y1 = y_max;
            x1 = (r-y1*normal_y)/normal_x;
        }

        x2 = x_max;
        y2 = (r-x2*normal_x)/normal_y;

        if (y2 < y_min)
        {
            y2 = y_min;
            x2 = (r-y2*normal_y)/normal_x;
        }
        else if (y2 > y_max)
        {
            y2 = y_max;
            x2 = (r-y2*normal_y)/normal_x;
        }
    }
    else
    {
        y1 = y_min;
        x1 = (r-y1*normal_y)/normal_x;

        if (x1 < x_min)
        {
            x1 = x_min;
            y1 = (r-x1*normal_x)/normal_y;
        }
        else if (x1 > x_max)
        {
            x1 = x_max;
            y1 = (r-x1*normal_x)/normal_y;
        }

        y2 = y_max;
        x2 = (r-y2*normal_y)/normal_x;

        if (x2 < x_min)
        {
            x2 = x_min;
            y2 = (r-x2*normal_x)/normal_y;
        }
        else if (x2 > x_max)
        {
            x2 = x_max;
            y2 = (r-x2*normal_x)/normal_y;
        }
    }
    *out_x1 = x1;
    *out_y1 = y1;
    *out_x2 = x2;
    *out_y2 = y2;
}

void
asci_line_to_segment(r32 t, r32 r, r32 x_min, r32 x_max, r32 y_min, r32 y_max, r32 *x1, r32 *y1, r32 *x2, r32 *y2)
// Computes the line endpoints from the (angle, distance) parametrization
{
    r32 normal_x = cos(t);
    r32 normal_y = sin(t);
    asci_line_to_segment(normal_x, normal_y, r, x_min, x_max, y_min, y_max, x1, y1, x2, y2);
}

void
asci_clip_line(vec2 *p1, vec2 *p2, vec2 n, r32 x_min, r32 x_max, r32 y_min, r32 y_max)
// Extends and/or clips the line segment (p1, p2) against the boundaries
{
    r32 normal_x = n.x;
    r32 normal_y = n.y;
    r32 r = p1->x*n.x + p1->y*n.y;
    asci_line_to_segment(normal_x, normal_y, r, x_min, x_max, y_min, y_max, &p1->x, &p1->y, &p2->x, &p2->y);
}

bool
asci_seg_seg_test(vec2 p1, vec2 p2, vec2 q1, vec2 q2, vec2 *p = 0, r32 *interpolant_q = 0)
{
    vec2 dp = p2-p1;
    vec2 dq = q2-q1;
    r32 dot = dq.x*dp.y - dq.y*dp.x;
    r32 eps = 0.001f;
    if (abs(dot) < eps)
        return false;

    vec2 pq = p1-q1;
    r32 v = (pq.x*dp.y - pq.y*dp.x)/dot;

    if (v < 0.0f || v > 1.0f)
        return false;

    vec2 qp = q1-p1;
    r32 u = -(qp.x*dq.y - qp.y*dq.x)/dot;
    if (u < 0.0f || u > 1.0f)
        return false;

    if (p)
        *p = q1 + v*dq;

    if (interpolant_q)
        *interpolant_q = v;

    return true;
}

#ifdef ASC_GRID_DEBUG
void asci_draw_line_pinhole(mat3 R, vec3 T, r32 f, r32 u0, r32 v0, r32 zn, vec3 p1, vec3 p2)
// p1, p2: In world space
// u0, v0: Image center
// zn, zf: Near and far clip planes
// T: p^w_{c/w}
// R: R^c_w
// Output y is 0 at top of screen at height at bottom of screen
{
    p1 = R*(p1 - T);
    p2 = R*(p2 - T);
    r32 z1 = p1.z;
    r32 z2 = p2.z;
    vec3 clip1 = p1;
    vec3 clip2 = p2;
    if (-z1 < zn)
    {
        r32 t = (-zn - z2) / (z1 - z2);
        clip1 = p2 + (p1 - p2)*t;
    }
    if (-z2 < zn)
    {
        r32 t = (-zn - z1) / (z2 - z1);
        clip2 = p1 + (p2 - p1)*t;
    }
    r32 u1 = -f*clip1.x/clip1.z + u0;
    r32 v1 = +f*clip1.y/clip1.z + v0;
    r32 u2 = -f*clip2.x/clip2.z + u0;
    r32 v2 = +f*clip2.y/clip2.z + v0;
    glVertex2f(u1, v1);
    glVertex2f(u2, v2);
}
#endif

asc_GridResult
asc_find_grid(
    u08 *gray,
    s32 width,
    s32 height,
    r32 camera_ex,
    r32 camera_ey,
    r32 camera_z,
    asc_GridOptions options,
    u08 *rgb)
{
    ASCI_ASSERT(width <= ASCI_MAX_WIDTH);
    ASCI_ASSERT(height <= ASCI_MAX_HEIGHT);

    r32 tile_width = options.tile_width;

    #ifdef ASC_GRID_DEBUG
    VDBB("Input (RGB)");
    {
        vdbOrtho(-1, +1, +1, -1);
        vdbSetTexture2D(0, rgb, width, height, GL_RGB);
        vdbDrawTexture2D(0);
    }
    VDBE();
    #endif

    #ifdef ASC_GRID_DEBUG
    VDBB("Input (GRAY)");
    {
        vdbOrtho(-1, +1, +1, -1);
        vdbSetTexture2D(0, gray, width, height, GL_LUMINANCE);
        vdbDrawTexture2D(0);
    }
    VDBE();
    #endif

    // TIMING("SOBEL");

    static asci_Feature features[ASCI_MAX_WIDTH*ASCI_MAX_HEIGHT];
    s32 feature_count = 0;
    asci_sobel(
        gray,
        width,
        height,
        options.sobel_threshold,
        features,
        &feature_count);

    if (feature_count == 0)
    {
        asc_GridResult result = {0};
        result.error = options.max_error;
        return result;
    }

    // TIMING("SOBEL");

    // TIMING("FISHEYE");
    if (options.correct_fisheye)
    {
        // Here I'm modifying the features array in place
        asci_fisheye_undistort(features, feature_count,
                               width, height,
                               features, &feature_count,
                               options.fisheye_f,
                               options.fisheye_center_x,
                               options.fisheye_center_y,
                               options.pinhole_fov_x);
    }
    // TIMING("FISHEYE");

    const s32 bins_t = 256;
    const s32 bins_r = 512;
    const r32 r_max = sqrt((r32)(width*width+height*height));
    const r32 r_min = -r_max;
    const r32 t_min = 0.0f;
    const r32 t_max = ASCI_PI;

    // EXTRACT LINES THROUGH STANDARD HOUGH TRANSFORM

    // TIMING("HOUGH");

    static asci_HoughCell prev_tile_maximas[4] = {0};
    static s32 prev_tile_maximas_count = 0;

    static asci_HoughCell histogram[bins_t*bins_r];
    s32 hough_peak_count = 0;
    asci_hough_transform(histogram, bins_t, bins_r, t_min, t_max, r_min, r_max, features, feature_count, &hough_peak_count);

    // TIMING("HOUGH");

    // TIMING("DILATE");
    static s32 dilated_counts[countof(histogram)];
    s32 width_t = 5;
    s32 width_r = 10;
    asci_dilate_histogram(histogram, dilated_counts, bins_t, bins_r, width_t, width_r);
    // TIMING("DILATE");

    // TIMING("MAXIMA");
    static asci_HoughCell maximas[128];
    s32 maxima_count = 0;
    asci_find_local_maxima(histogram, dilated_counts, bins_t, bins_r, width_t, width_r, options.maxima_threshold, countof(maximas), maximas, &maxima_count);
    // TIMING("MAXIMA");

    #ifdef ASC_GRID_DEBUG
    {
        r32 tx = 0.0f;
        r32 ty = 0.0f;
        r32 ex = camera_ex;
        r32 ey = camera_ey;
        r32 ez = 0.0f;
        r32 hz = camera_z;
        r32 f = (width/2.0f) / tan(options.pinhole_fov_x/2.0f);
        r32 u0 = options.pinhole_center_x;
        r32 v0 = options.pinhole_center_y;

        VDBB("World features");
        {
            mat3 R = m_mat3(mat_rotate_z(ez)*mat_rotate_y(ey)*mat_rotate_x(ex));

            static r32 range = 1.0f;
            vdbOrtho(-range*vdbAspect(), +range*vdbAspect(), -range, +range);
            glPoints(4.0f);
            {
                glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
                forio(feature_count)
                {
                    vec2 uv = m_vec2(features[i].x, features[i].y);
                    vec2 XY = asci_trace_world_plane(f, u0, v0, R, hz, uv) + m_vec2(tx, ty);
                    glVertex2f(XY.x, XY.y);
                    i += asci_xor128() % 5;
                }
            }
            glEnd();

            SliderFloat("tx", &tx, -1.0f, +1.0f);
            SliderFloat("ty", &ty, -1.0f, +1.0f);
            SliderFloat("ex", &ex, -0.4f, +0.4f);
            SliderFloat("ey", &ey, -0.4f, +0.4f);
            SliderFloat("ez", &ez, -0.4f, +0.4f);
            SliderFloat("hz", &hz, +0.1f, +3.0f);
            SliderFloat("range", &range, 1.0f, 20.0f);

        }
        VDBE();
    }
    #endif

    #ifdef _ASC_GRID_DEBUG
    VDBB("Hough");
    {
        s32 mouse_x = (0.5f+0.5f*MOUSEX)*bins_t;
        s32 mouse_y = (0.5f+0.5f*MOUSEY)*bins_r;
        vdbOrtho(0.0f, bins_t, 0.0f, bins_r);
        glPoints(4.0f);
        foryo(bins_r)
        forxo(bins_t)
        {
            s32 c_original = histogram[y*bins_t+x].count;
            s32 c_dilated = dilated_counts[y*bins_t+x];
            if (c_original < 3) continue;
            vdbColorRamp(c_original / (r32)hough_peak_count);
            glVertex2f(x+0.5f, y+0.5f);

            if (x == mouse_x && y == mouse_y)
            {
                SetTooltip("ti: %d ri: %d\noriginal: %d dilated: %d", x, y, c_original, c_dilated);
            }
        }
        glEnd();
    }
    VDBE();
    #endif

    #ifdef ASC_GRID_DEBUG
    VDBB("Hough maxima");
    {
        vdbOrtho(0.0f, width, height, 0.0f);
        glPoints(4.0f);
        glColor4f(0.1f, 0.1f, 0.1f, 1.0f);
        forio(feature_count) glVertex2f(features[i].x, features[i].y);
        glEnd();

        r32 min_mouse_d = FLT_MAX;
        s32 min_mouse_i = 0;

        r32 mouse_t = m_map(-1.0f, +1.0f, MOUSEX, t_min, t_max);
        r32 mouse_r = m_map(-1.0f, +1.0f, MOUSEY, r_min, r_max);

        vdbOrtho(t_min, t_max, r_min, r_max);
        glPoints(8.0f);
        forio(maxima_count)
        {
            r32 t = maximas[i].avg_t;
            r32 r = maximas[i].avg_r;
            s32 c = maximas[i].count;
            vdbColorRamp(c / (r32)hough_peak_count);

            r32 t_ndc = -1.0f + 2.0f * (t-t_min)/(t_max-t_min);
            r32 r_ndc = -1.0f + 2.0f * (r-r_min)/(r_max-r_min);
            r32 mouse_d = (t_ndc-MOUSEX)*(t_ndc-MOUSEX) + (r_ndc-MOUSEY)*(r_ndc-MOUSEY);
            if (mouse_d < min_mouse_d)
            {
                min_mouse_d = mouse_d;
                mouse_t = t;
                mouse_r = r;
                SetTooltip("%d %d\n%.2f %.2f", i, c, t, r);
            }
            glVertex2f(t, r);
        }

        glColor4f(1.0f, 1.0f, 0.2f, 1.0f);
        glVertex2f(mouse_t, mouse_r);

        glColor4f(1.0f, 0.2f, 0.1f, 1.0f);
        forio(prev_tile_maximas_count)
            glVertex2f(prev_tile_maximas[i].avg_t, prev_tile_maximas[i].avg_r);
        glEnd();

        r32 x1, y1, x2, y2;
        asci_line_to_segment(mouse_t, mouse_r, 0.0f, width, 0.0f, height, &x1, &y1, &x2, &y2);

        vdbOrtho(0.0f, width, height, 0.0f);
        glLineWidth(4.0f);
        glBegin(GL_LINES);
        glColor4f(1.0f, 0.2f, 0.2f, 1.0f);
        glVertex2f(x1, y1);
        glVertex2f(x2, y2);
        glEnd();
    }
    VDBE();
    #endif

    // SORT LOCAL MAXIMAS BY HOUGH VOTE COUNT AND
    // KEEP A SUBSET OF THE MOST STRONG VOTES
    qsort(maximas, maxima_count, sizeof(asci_HoughCell), asci_cmp_rt);
    #define ASCI_GRID_MAXIMA_LIMIT 20
    if (maxima_count > ASCI_GRID_MAXIMA_LIMIT)
        maxima_count = ASCI_GRID_MAXIMA_LIMIT;

    // FILTER WITH PREVIOUS FRAME MAXIMAS
    #define ASCI_GRID_ENABLE_LPF
    #ifdef ASCI_GRID_ENABLE_LPF
    #define ASCI_GRID_LPF_GAIN_T    (0.65f)
    #define ASCI_GRID_LPF_GAIN_R    (0.65f)
    #define ASCI_GRID_LPF_PROXIMITY (0.1f)
    forio(prev_tile_maximas_count)
    {
        r32 t1 = prev_tile_maximas[i].avg_t;
        r32 r1 = prev_tile_maximas[i].avg_r;
        r32 t1_ndc = -1.0f + 2.0f * (t1-t_min)/(t_max-t_min);
        r32 r1_ndc = -1.0f + 2.0f * (r1-r_min)/(r_max-r_min);

        // Find the closest new maxima that is also sufficiently close
        r32 d0 = ASCI_GRID_LPF_PROXIMITY*ASCI_GRID_LPF_PROXIMITY;
        s32 closest_j = -1;
        r32 closest_d = -1.0f;
        forjo(maxima_count)
        {
            r32 t2 = maximas[j].avg_t;
            r32 r2 = maximas[j].avg_r;
            r32 t2_ndc = -1.0f + 2.0f * (t2-t_min)/(t_max-t_min);
            r32 r2_ndc = -1.0f + 2.0f * (r2-r_min)/(r_max-r_min);

            r32 dt = (t2_ndc-t1_ndc);
            r32 dr = (r2_ndc-r1_ndc);
            r32 d = dt*dt + dr*dr;
            if ((d < d0) && (d < closest_d || closest_d < 0.0f))
            {
                closest_j = j;
                closest_d = d;
            }
        }

        // Moving average filter
        if (closest_j >= 0)
        {
            r32 t2 = maximas[closest_j].avg_t;
            r32 r2 = maximas[closest_j].avg_r;
            r32 kt = ASCI_GRID_LPF_GAIN_T;
            r32 kr = ASCI_GRID_LPF_GAIN_R;
            maximas[closest_j].avg_t = kt*t2 + (1.0f - kt)*t1;
            maximas[closest_j].avg_r = kr*r2 + (1.0f - kr)*r1;
        }
    }
    #endif

    // COMPUTE LINE ENDPOINTS IN IMAGE SPACE
    struct Line
    {
        vec2 p1;
        vec2 p2;
        vec2 n;
        r32 weight;
        r32 cost;
    };
    Line lines[countof(maximas)];
    forio(maxima_count)
    {
        r32 r = maximas[i].avg_r;
        r32 t = maximas[i].avg_t;
        s32 n = 2*width_r+2*width_t+2;
        r32 a = maximas[i].count / (r32)n; // Average Hough count in neighborhood
        r32 w = a / (r32)hough_peak_count; // Normalized Hough count as a metric for line saliency
        r32 x1, y1, x2, y2;
        asci_line_to_segment(t, r, 0.0f, width, 0.0f, height, &x1, &y1, &x2, &y2);
        lines[i].p1 = m_vec2(x1, y1);
        lines[i].p2 = m_vec2(x2, y2);
        lines[i].weight = w;
        lines[i].cost = 0.0f;
    }

    // DRAW WORLD TRANSFORMED LINES
    #ifdef _ASC_GRID_DEBUG
    {
        r32 f = (width/2.0f) / (tan(options.pinhole_fov_x/2.0f));
        r32 ex = camera_ex;
        r32 ey = camera_ey;
        r32 ez = 0.0f;
        r32 u0 = options.pinhole_center_x;
        r32 v0 = options.pinhole_center_y;
        VDBB("World transform");
        {
            static r32 range = 0.1f;
            vdbOrtho(-range*vdbAspect(), +range*vdbAspect(), -range, +range);
            glLines(4.0f);
            mat3 R = m_mat3(mat_rotate_z(ez)*mat_rotate_y(ey)*mat_rotate_x(ex));
            forio(maxima_count)
            {
                vec2 XY1 = asci_trace_world_plane(f, u0, v0, R, camera_z, lines[i].p1);
                vec2 XY2 = asci_trace_world_plane(f, u0, v0, R, camera_z, lines[i].p2);
                glVertex2f(XY1);
                glVertex2f(XY2);
            }
            glEnd();

            SliderFloat("hfov", &options.pinhole_fov_x, 0.5f, 3.1f);
            SliderFloat("camera_ex", &ex, -0.6f, +0.6f);
            SliderFloat("camera_ey", &ey, -0.6f, +0.6f);
            SliderFloat("ez", &ez, -0.7f, +0.7f);
            SliderFloat("hz", &camera_z, +0.1f, 3.0f);
            SliderFloat("range", &range, 1.0f, 20.0f);
        }
        VDBE();
    }
    #endif

    // TRANSFORM LINE ENDPOINTS TO WORLD SPACE
    // @ CLIPPED TRANSFORM
    {
        r32 f = (width/2.0f) / (tan(options.pinhole_fov_x/2.0f));
        r32 ex = camera_ex;
        r32 ey = camera_ey;
        r32 ez = 0.0f;
        r32 u0 = options.pinhole_center_x;
        r32 v0 = options.pinhole_center_y;
        mat3 R = m_mat3(mat_rotate_z(ez)*mat_rotate_y(ey)*mat_rotate_x(ex));
        forio(maxima_count)
        {
            vec2 XY1 = asci_trace_world_plane(f, u0, v0, R, camera_z, lines[i].p1);
            vec2 XY2 = asci_trace_world_plane(f, u0, v0, R, camera_z, lines[i].p2);
            lines[i].p1 = XY1;
            lines[i].p2 = XY2;
            vec2 tangent = m_normalize(XY2-XY1);
            lines[i].n = m_vec2(-tangent.y, tangent.x);
        }
    }

    // GENERATE EXTRA LINES FOR LOW ALTITUDE SCENARIO
    #if 1
    if (maxima_count <= 6)
    {
        s32 j = maxima_count;
        forio(maxima_count)
        {
            if (j+3 >= 25)
                break;

            Line line1 = lines[i];
            Line line2 = line1;
            line2.p1 += tile_width*line1.n;
            line2.p2 += tile_width*line1.n;
            line2.cost = ASCI_GRID_PREDICTED_LINE_COST;

            Line line3 = line1;
            line3.n.x = -line1.n.y;
            line3.n.y =  line1.n.x;
            line3.p1 = 0.5f*(line1.p1+line1.p2) - 0.5f*tile_width*line3.n;
            line3.p2 = 0.5f*(line2.p1+line2.p2) - 0.5f*tile_width*line3.n;
            line3.cost = ASCI_GRID_PREDICTED_LINE_COST;

            Line line4 = line1;
            line4.n.x = -line1.n.y;
            line4.n.y =  line1.n.x;
            line4.p1 = 0.5f*(line1.p1+line1.p2) + 0.5f*tile_width*line4.n;
            line4.p2 = 0.5f*(line2.p1+line2.p2) + 0.5f*tile_width*line4.n;
            line4.cost = ASCI_GRID_PREDICTED_LINE_COST;

            lines[j++] = line2;
            lines[j++] = line3;
            lines[j++] = line4;
        }
        maxima_count = j;
    }
    #endif

    // CLIP LINES AGAINST WORLD BOUNDARIES
    forio(maxima_count)
    {
        // TODO: Resulting length might be zero!
        asci_clip_line(&lines[i].p1, &lines[i].p2, lines[i].n, -10.0f, +10.0f, -10.0f, +10.0f);
    }

    #ifdef _ASC_GRID_DEBUG
    VDBB("Lines");
    {
        vdbOrtho(-2.0f*vdbAspect(), +2.0f*vdbAspect(), -2.0f, +2.0f);
        forio(maxima_count)
        {
            glLines(4.0f);
            if (lines[i].cost > 0.0f)
                glColor4f(1.0f, 0.2f, 0.1f, 1.0f);
            else
                glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
            glVertex2f(lines[i].p1.x, lines[i].p1.y);
            glVertex2f(lines[i].p2.x, lines[i].p2.y);
            glEnd();
        }

        Text("%d", maxima_count);
    }
    VDBE();
    #endif

    // EXHAUSTIVE SEARCH FOR BEST GRID TILE
    struct Tile
    {
        s32 i1, i2, i3, i4;
        r32 e;
        vec2 x1, x2, x3, x4, xm;
    };

    Tile best = {0};
    best.e = options.max_error;
    for (s32 i1 = 0;    i1 < maxima_count; i1++)
    for (s32 i2 = i1+1; i2 < maxima_count; i2++)
    for (s32 i3 = i2+1; i3 < maxima_count; i3++)
    for (s32 i4 = i3+1; i4 < maxima_count; i4++)
    {
        vec2 P1[] = { lines[i1].p1, lines[i2].p1, lines[i3].p1, lines[i4].p1 };
        vec2 P2[] = { lines[i1].p2, lines[i2].p2, lines[i3].p2, lines[i4].p2 };
        vec2 N[] = { lines[i1].n, lines[i2].n, lines[i3].n, lines[i4].n };

        // We compute a measure of scale correctness by
        // comparing the average length of the sides of
        // the tile with the ideal length (i.e. 1 meter)
        r32 e_r = 0.0f; // Average absolute L1 error from ideal length
        vec2 X[4]; // Tile vertices
        vec2 xm = m_vec2(0.0f, 0.0f); // Tile center
        {
            s32 intersection_count = 0;
            // Compute intersections
            // @ Computing a tile from a set of four lines is
            // not trivial! Need to consider multiple intersections.
            // Correctly clipping by world bounds will help, but
            // the problem still remains!!
            for (s32 i = 0; i < 4; i++)
            for (s32 j = i+1; j < 4; j++)
            {
                vec2 x;
                if (asci_seg_seg_test(P1[i], P2[i], P1[j], P2[j], &x) && intersection_count < 4)
                {
                    X[intersection_count++] = x;
                }
            }
            if (intersection_count < 4) continue; // Did not find enough intersection points

            // Compute center point (Used later to compute x and y results)
            for (s32 i = 0; i < 4; i++)
                xm += 0.25f*X[i];

            // Link the intersection points by tile-connectivity
            vec2 X_sorted[4];
            r32 d1 = m_length(X[1]-X[0]);
            r32 d2 = m_length(X[2]-X[0]);
            r32 d3 = m_length(X[3]-X[0]);

            if (d3 > d1 && d3 > d2) // d3 is furthest away
            {
                X_sorted[0] = X[0];
                X_sorted[1] = X[1];
                X_sorted[2] = X[3];
                X_sorted[3] = X[2];
            }

            if (d2 > d3 && d2 > d1) // d2 is furthest away
            {
                X_sorted[0] = X[0];
                X_sorted[1] = X[1];
                X_sorted[2] = X[2];
                X_sorted[3] = X[3];
            }

            if (d1 > d3 && d1 > d2) // d1 is furthest away
            {
                X_sorted[0] = X[0];
                X_sorted[1] = X[3];
                X_sorted[2] = X[1];
                X_sorted[3] = X[2];
            }

            for (s32 i = 0; i < 4; i++)
                X[i] = X_sorted[i];

            // Neighboring intersections should be close to a tile-widths distance
            e_r += 0.25f*fabs(m_length(X[1]-X[0])-tile_width);
            e_r += 0.25f*fabs(m_length(X[2]-X[1])-tile_width);
            e_r += 0.25f*fabs(m_length(X[3]-X[2])-tile_width);
            e_r += 0.25f*fabs(m_length(X[0]-X[3])-tile_width);
        }

        // Compute a measure of angle correctness
        // (The sides should be pairwise parallel, and otherwise orthogonal)
        r32 e_t = 0.0f;
        {
            vec2 S1 = X[1]-X[0]; r32 L1 = m_length(S1);
            vec2 S2 = X[2]-X[1]; r32 L2 = m_length(S2);
            vec2 S3 = X[3]-X[2]; r32 L3 = m_length(S3);
            vec2 S4 = X[0]-X[3]; r32 L4 = m_length(S4);
            r32 a1 = fabs(m_dot(S1, S2)) / (L1*L2); e_t = a1;
            r32 a2 = fabs(m_dot(S2, S3)) / (L2*L3); if (a2 > e_t) e_t = a2;
            r32 a3 = fabs(m_dot(S3, S4)) / (L3*L4); if (a3 > e_t) e_t = a3;
            r32 a4 = fabs(m_dot(S4, S1)) / (L4*L1); if (a4 > e_t) e_t = a4;
            r32 a5 = 1.0f - fabs(m_dot(S1, S3)) / (L1*L3); if (a5 > e_t) e_t = a5;
            r32 a6 = 1.0f - fabs(m_dot(S2, S4)) / (L2*L4); if (a6 > e_t) e_t = a6;
        }

        r32 e_prior = lines[i1].cost + lines[i2].cost + lines[i3].cost + lines[i4].cost;

        r32 lambda = 2.5f;
        r32 e = lambda*e_t + e_r + e_prior;

        Tile tile = {0};
        tile.i1 = i1;
        tile.i2 = i2;
        tile.i3 = i3;
        tile.i4 = i4;
        tile.x1 = X[0];
        tile.x2 = X[1];
        tile.x3 = X[2];
        tile.x4 = X[3];
        tile.xm = xm;
        tile.e = e;

        if (tile.e < best.e)
        {
            best = tile;
            #ifdef ASC_GRID_DEBUG
            VDBB("Tile quality");
            {
                // DRAW FEATURES IN IMAGE SPACE
                vdbOrtho(0.0f, width, height, 0.0f);
                glPoints(4.0f);
                glColor4f(0.75f, 0.7f, 0.65f, 1.0f);
                forio(feature_count) glVertex2f(features[i].x, features[i].y);
                glEnd();

                // WORLD TO CAMERA SPACE TRANSFORM PARAMETERS
                mat3 R = m_transpose(m_mat3(mat_rotate_y(camera_ey)*mat_rotate_x(camera_ex)));
                vec3 T = m_vec3(0.0f, 0.0f, camera_z);
                r32 f = (width/2.0f) / tan(options.pinhole_fov_x/2.0f);
                r32 zn = 0.01f;

                #define DRAW(A, B) \
                    asci_draw_line_pinhole(R, T, f, \
                                       options.pinhole_center_x, \
                                       options.pinhole_center_y, \
                                       zn, m_vec3(A.x, A.y, 0.0f), m_vec3(B.x, B.y, 0.0f))

                // DRAW LINES TRANSFORMED INTO IMAGE SPACE
                vdbOrtho(0.0f, width, height, 0.0f);
                glLines(4.0f);
                glColor4f(0.1f, 0.2f, 1.0f, 1.0f);
                forio(4) DRAW(P1[i], P2[i]);

                // DRAW TILE EDGES TRANSFORMED INTO IMAGE SPACE
                glColor4f(1.0f, 0.2f, 0.1f, 1.0f);
                DRAW(tile.x1, tile.x2);
                DRAW(tile.x2, tile.x3);
                DRAW(tile.x3, tile.x4);
                DRAW(tile.x4, tile.x1);
                glEnd();
                #undef DRAW

                Text("e_t: %.5f\ne_r: %.5f\ne: %.5f", e_t, e_r, e);
                Text("e_prior: %.5f", e_prior);
            }
            VDBE();
            #endif
        }
    }

    asc_GridResult result = {0};
    result.error = options.max_error;
    result.height = camera_z;
    prev_tile_maximas_count = 0;

    // #define ASCI_GRID_LOW_ALTITUDE_SUPPORT
    #ifdef ASCI_GRID_LOW_ALTITUDE_SUPPORT
    if (best.e >= options.max_error)
    {
        // Can we find two perpendicular lines?
        bool two_perps = false;
        r32 best_pair_e = 0.0f;
        s32 best_pair_i = 0;
        s32 best_pair_j = 0;
        bool found = false;
        forio(maxima_count)
        {
            for (s32 j = i+1; j < maxima_count; j++)
            {
                r32 e = m_dot(lines[i].n, lines[j].n);
                if (e < best_pair_e || !found)
                {
                    best_pair_e = e;
                    best_pair_i = i;
                    best_pair_j = j;
                    found = true;
                }
            }
        }

        if (found && best_pair_e < 0.2f)
        {
            // best.i1 = best_pair_i;
            // best.i2 = best_pair_j;
            // best.i3 = i3;
            // best.i4 = i4;
            // best.e = 0.95f*options_max_error;

            #ifdef ASC_GRID_DEBUG
            VDBB("Lines");
            {
                vdbOrtho(-2.0f*vdbAspect(), +2.0f*vdbAspect(), -2.0f, +2.0f);
                glLines(4.0f);
                glColor4f(1, 1, 1, 1);
                glLine2f(lines[best_pair_i].p1, lines[best_pair_i].p2);
                glLine2f(lines[best_pair_j].p1, lines[best_pair_j].p2);
                glEnd();

                Text("%d", maxima_count);
            }
            VDBE();
            #endif
        }
        else
        {
            // Could not find a pair of orthogonal lines.
            // Let's assume one or both of the components
            // are located in the middle of the tile. We
            // find a single closest line to the image
            // center, and compute the other component
            // from this (if we find such a line).
            #ifdef ASC_GRID_DEBUG
            VDBB("Error");
            {
                vdbOrtho(-1.0f, +1.0f, +1.0f, -1.0f);
                vdbImage(gray, width, height, GL_LUMINANCE);
                Text("Error was too large");
            }
            VDBE();
            #endif
        }
    }
    #endif

    if (best.e < options.max_error)
    {
        // Remember these lines for the next frame
        prev_tile_maximas[0] = maximas[best.i1];
        prev_tile_maximas[1] = maximas[best.i2];
        prev_tile_maximas[2] = maximas[best.i3];
        prev_tile_maximas[3] = maximas[best.i4];
        prev_tile_maximas_count = 4;

        // Finally, given the tile intersection points, we compute
        // the four possible poses by considering the four possible
        // yaw angles.

        // Compute the four angles, arbitrarily choosing the first
        // line in the list as the base angle.
        r32 yaw1 = atan2(lines[best.i1].n.y, lines[best.i1].n.x);
        r32 yaw2 = asci_angle(yaw1 + ASCI_PI/2.0f);
        r32 yaw3 = asci_angle(yaw2 + ASCI_PI/2.0f);
        r32 yaw4 = asci_angle(yaw3 + ASCI_PI/2.0f);

        // Compute the x, y location corresponding to each angle
        r32 yaws[4] = { yaw1, yaw2, yaw3, yaw4 };
        for (s32 i = 0; i < countof(yaws); i++)
        {
            r32 yaw = yaws[i];
            r32 c = cos(-yaw);
            r32 s = sin(-yaw);
            r32 xr = c*best.xm.x - s*best.xm.y;
            r32 yr = s*best.xm.x + c*best.xm.y;
            r32 cx = asci_modulo(tile_width/2.0f - xr, tile_width);
            r32 cy = asci_modulo(tile_width/2.0f - yr, tile_width);

            result.x[i] = cx;
            result.y[i] = cy;
            result.yaw[i] = -yaw; // The measured yaw angle is the grid
                                  // relative the camera frame, but we
                                  // want the camera frame relative the
                                  // grid.
        }

        // Refine the height estimate by comparing the length of the
        // sides of the tile with the ideal length (i.e. 1 meter).
        // The error is used to drive an adaptation law.
        // if (best.e < 0.5f*options.max_error) // Only perform adaptation if error was small enough
        {
            // Average ratio between measured lengths and ideal lengths
            r32 el = 0.0f;
            el += 0.25f*m_length(best.x2-best.x1)/tile_width;
            el += 0.25f*m_length(best.x3-best.x2)/tile_width;
            el += 0.25f*m_length(best.x4-best.x3)/tile_width;
            el += 0.25f*m_length(best.x1-best.x4)/tile_width;

            // If el = 1: The new height will be identical to the old height
            // If el > 1: The predicted sidelengths are shorter than observed,
            //            implying that the height should decrease.
            // If el < 1: The predicted sidelengths are longer than observed,
            //            implying that the height should increase.
            r32 camera_z_hat = camera_z/el;

            r32 k = options.height_adaptation_rate;
            result.height = (1.0f - k)*camera_z + k*camera_z_hat;
        }
        // else
        // {
        //     result.height = camera_z;
        // }

        result.error = best.e;
    }

    // DRAW GRID ONTOP OF FISHEYE IMAGE
    #ifdef _ASC_GRID_DEBUG
    {
        r32 ex = camera_ex;
        r32 ey = camera_ey;
        r32 ez = result.yaw[0];
        r32 f = options.fisheye_f;
        r32 u0 = options.fisheye_center_x;
        r32 v0 = options.fisheye_center_y;
        vec3 p_cam = m_vec3(result.x[0], result.y[0], camera_z);
        VDBB("Fisheye grid");
        {
            vdbOrtho(-1.0f, +1.0f, +1.0f, -1.0f);
            if (rgb) vdbImage(rgb, width, height, GL_RGB);
            else vdbImage(gray, width, height, GL_LUMINANCE);

            vdbOrtho(0.0f, width, height, 0.0f);
            mat3 c_to_w = m_mat3(mat_rotate_z(ez)*mat_rotate_y(ey)*mat_rotate_x(ex));
            mat3 w_to_c = m_transpose(c_to_w);
            glLines(2.0f);
            glColor4f(COLOR_YELLOW);
            forio(21)
            {
                vdbDrawLineFisheye(w_to_c, p_cam, f, u0, v0,
                             m_vec3(-10.0f, -10.0f + i, 0.0f),
                             m_vec3(+10.0f, -10.0f + i, 0.0f), 256);
                vdbDrawLineFisheye(w_to_c, p_cam, f, u0, v0,
                             m_vec3(-10.0f + i, -10.0f, 0.0f),
                             m_vec3(-10.0f + i, +10.0f, 0.0f), 256);
            }
            glEnd();

            SliderAngle("ex", &ex, -45.0f, +45.0f);
            SliderAngle("ey", &ey, -45.0f, +45.0f);
            SliderAngle("ez", &ez, -45.0f, +45.0f);
            SliderFloat("x", &p_cam.x, -2.0f, +2.0f);
            SliderFloat("y", &p_cam.y, -2.0f, +2.0f);
            SliderFloat("z", &p_cam.z, +0.1f, +5.0f);
            Separator();

            Text("Quality of grid estimate");
            Text("e: %.5f", result.error);
            Separator();

            Text("Estimated height");
            Text("z: %.5f meters", camera_z);
            Text("z_hat: %.5f meters", result.height);
        }
        VDBE();
    }
    #endif

    // DRAW GRID ONTOP OF (DEWARPED) FEATURES
    #ifdef ASC_GRID_DEBUG
    {
        int solution = 0;
        r32 euler_z = result.yaw[solution];
        r32 cx = result.x[solution];
        r32 cy = result.y[solution];
        VDBB("Grid estimation quality");
        {
            vdbOrtho(0.0f, width, height, 0.0f);
            glPointSize(4.0f);
            glBegin(GL_POINTS);
            {
                for (int i = 0; i < feature_count; i++)
                {
                    r32 x = (r32)features[i].x;
                    r32 y = (r32)features[i].y;

                    glColor4f(0.75f, 0.7f, 0.65f, 1.0f);
                    glVertex2f(x, y);
                }
            }
            glEnd();

            mat3 R = m_transpose(m_mat3(mat_rotate_z(euler_z)*mat_rotate_y(camera_ey)*mat_rotate_x(camera_ex)));
            vec3 T = m_vec3(cx, cy, camera_z);
            r32 f = (width/2.0f) / tan(options.pinhole_fov_x/2.0f);

            vdbOrtho(0.0f, width, height, 0.0f);
            glLines(4.0f);
            glColor4f(1.0f, 0.2f, 0.1f, 1.0f);
            forio(21)
            {
                vec3 p1;
                vec3 p2;
                {
                    p1.x = -10.0f + i*tile_width;
                    p2.x = -10.0f + i*tile_width;
                    p1.y = -10.0f;
                    p2.y = +10.0f;
                    p1.z = 0.0f;
                    p2.z = 0.0f;
                    asci_draw_line_pinhole(R, T, f, options.pinhole_center_x, options.pinhole_center_y, 0.01f, p1, p2);
                }
                {
                    p1.y = -10.0f + i*tile_width;
                    p2.y = -10.0f + i*tile_width;
                    p1.x = -10.0f;
                    p2.x = +10.0f;
                    p1.z = 0.0f;
                    p2.z = 0.0f;
                    asci_draw_line_pinhole(R, T, f, options.pinhole_center_x, options.pinhole_center_y, 0.01f, p1, p2);
                }
            }
            glEnd();

            #if 1
            glViewport(0, 0, 200, 200);
            vdbOrtho(-0.1f, +1.1f, -0.1f, +1.1f);
            glBegin(GL_QUADS);
            glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
            glVertex2f(-0.1f, -0.1f);
            glVertex2f(+1.1f, -0.1f);
            glVertex2f(+1.1f, +1.1f);
            glVertex2f(-0.1f, +1.1f);
            glEnd();

            glLines(4.0f);
            glColor4f(1, 1, 1, 1);
            glVertex2f(0.0f, 0.0f);
            glVertex2f(1.0f, 0.0f);
            glVertex2f(1.0f, 0.0f);
            glVertex2f(1.0f, 1.0f);
            glVertex2f(1.0f, 1.0f);
            glVertex2f(0.0f, 1.0f);
            glVertex2f(0.0f, 1.0f);
            glVertex2f(0.0f, 0.0f);
            glEnd();

            glPoints(8.0f);
            glColor4f(1.0f, 0.2f, 0.1f, 1.0f);
            glVertex2f(result.x[0], result.y[0]);
            glVertex2f(result.x[1], result.y[1]);
            glVertex2f(result.x[2], result.y[2]);
            glVertex2f(result.x[3], result.y[3]);
            glEnd();

            glViewport(0, 0, vdbWidth(), vdbHeight());
            #endif

            #if 1
            Text("Yaw: %.2f", result.yaw[solution]*180.0f/ASCI_PI);
            Text("X: %.2f", result.x[solution]);
            Text("Y: %.2f", result.y[solution]);
            Text("Z: %.5f", camera_z);

            RadioButton("Solution 1", &solution, 0);
            RadioButton("Solution 2", &solution, 1);
            RadioButton("Solution 3", &solution, 2);
            RadioButton("Solution 4", &solution, 3);
            #endif

            // Calibrate result
            SliderFloat("cx", &cx, result.x[solution]-0.5f, result.x[solution]+0.5f);
            SliderFloat("cy", &cy, result.y[solution]-0.5f, result.y[solution]+0.5f);
            SliderFloat("cz", &camera_z, 0.8f, 1.5f);
            SliderFloat("ex", &camera_ex, -15.0f*ASCI_PI/180.0f, +15.0f*ASCI_PI/180.0f);
            SliderFloat("ey", &camera_ey, -15.0f*ASCI_PI/180.0f, +15.0f*ASCI_PI/180.0f);
            SliderFloat("ez", &euler_z, result.yaw[solution]-10.0f*ASCI_PI/180.0f, result.yaw[solution]+10.0f*ASCI_PI/180.0f);
        }
        VDBE();
    }
    #endif

    return result;
}

#endif
