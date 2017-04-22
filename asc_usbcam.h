// V4L2 video picture grabber
// Copyright (C) 2009 Mauro Carvalho Chehab <mchehab@infradead.org>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation version 2 of the License.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// Modified by Ascend NTNU:
// * H264 streaming for Logitech C920
// * Commentary
// * Rewrote from C to C++
// * Output to single file instead of frame by frame
// * Releasing on-device allocated memory by calling VIDIOC_REQBUF
//  with count 0
// * Setting the framerate

// How to compile
// --------------------------------------------------------
// Acquire the video 4 linux 2 development libraries (v4l2)
//   $ sudo apt-get install libv4l-dev
//   $ sudo apt-get install v4l-utils
//
// Acquire the jpeg turbo library:
//   $ git clone https://github.com/libjpeg-turbo/libjpeg-turbo
//   $ cd libjpeg-turbo
//   $ mkdir build
//   $ autoreconf -fiv
//   $ cd build
//   $ sh ../configure
//   $ make
//   $ make install prefix=/usr/local libdir=/usr/local/lib64
// See https://github.com/libjpeg-turbo/libjpeg-turbo/blob/master/BUILDING.md
// if that didn't work for you.

// To compile with g++ link with -lv4l2 and -lturbojpeg

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <libv4l2.h>
#include <turbojpeg.h>
#include <assert.h>

// Wrapper around v4l2_ioctl for programming the video device,
// that automatically retries the USB request if something
// unintentionally aborted the request.
void xioctl(int fh, int request, void *arg)
{
    int r;
    do
    {
        r = v4l2_ioctl(fh, request, arg);
    } while (r == -1 && ((errno == EINTR) || (errno == EAGAIN)));

    if (r == -1)
    {
        printf("USB request failed %d, %s\n", errno, strerror(errno));
        exit(EXIT_FAILURE);
    }
}

struct usb_Buffer
{
    void *start;
    size_t length;
};

struct usb_State
{
    bool initialized;

    int fd;
    int num_buffers;
    tjhandle decompressor;
    usb_Buffer buffers[4];
};

static usb_State usb_state = {0};

void usb_init(int width, int height, const char *device, int fps, int mmap_buffers = 3)
{
    // Open the device
    int fd = v4l2_open(device, O_RDWR | O_NONBLOCK, 0);
    if (fd < 0)
    {
        printf("Failed to open device\n");
        exit(EXIT_FAILURE);
    }

    // Specify the format of the data we want from the camera
    // Run v4l2-ctl --device=/dev/video1 --list-formats on the
    // device to see that sort of pixel formats are supported!
    v4l2_format fmt = {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
    xioctl(fd, VIDIOC_S_FMT, &fmt);

    // Set streaming parameters, i.e. frames per second.
    // You'll want to query the device for whether or not it
    // supports setting the frame time, and what possible choices
    // it supports.
    // See http://stackoverflow.com/questions/13981933/v4l2-fcntl-ioctl-vidioc-s-parm-for-setting-fps-and-resolution-of-camera-capture
    v4l2_streamparm parm = {};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.output.timeperframe.numerator = 1;
    parm.parm.output.timeperframe.denominator = fps;
    xioctl(fd, VIDIOC_S_PARM, &parm);

    // Sidenote: Run v4l-info /dev/video1 if you want to see what
    // other stuff that the device supports.

    int got_width = fmt.fmt.pix.width;
    int got_height = fmt.fmt.pix.height;
    int got_format = fmt.fmt.pix.pixelformat;

    assert(got_width == width);
    assert(got_height == height);
    assert(got_format == V4L2_PIX_FMT_MJPEG);

    // Request N buffers that are memory mapped between
    // our application space and the device
    v4l2_requestbuffers request = {};
    request.count = mmap_buffers;
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = V4L2_MEMORY_MMAP;
    xioctl(fd, VIDIOC_REQBUFS, &request);

    int num_buffers = request.count;

    // Check what format we _actually_ got
    #if 0
    printf("Opened device %s with format:\n", device);
    printf("\tWidth: %d\n", fmt.fmt.pix.width);
    printf("\tHeight: %d\n", fmt.fmt.pix.height);
    printf("\tPixel format: 0x%x\n", fmt.fmt.pix.pixelformat);
    printf("\tBuffers: %d\n", num_buffers);
    #else
    printf("\nOpened device %s\n", device);
    #endif

    static usb_Buffer buffers[4];
    assert(num_buffers <= 4);

    // Find out where each requested buffer is located in memory
    // and map them into our application space
    for (int buffer_index = 0; buffer_index < num_buffers; ++buffer_index)
    {
        v4l2_buffer buf = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = buffer_index;
        xioctl(fd, VIDIOC_QUERYBUF, &buf);

        buffers[buffer_index].length = buf.length;
        buffers[buffer_index].start =
                mmap(0 /* start anywhere */,
                     buf.length,
                     PROT_READ | PROT_WRITE /* required */,
                     MAP_SHARED /* recommended */,
                     fd, buf.m.offset);

        if (MAP_FAILED == buffers[buffer_index].start)
        {
            printf("mmap failed %d, %s\n", errno, strerror(errno));
            exit(EXIT_FAILURE);
        }
    }

    // Queue the buffers, i.e. indicate to the device
    // that they are available for writing now.
    for (int i = 0; i < num_buffers; ++i)
    {
        v4l2_buffer buf = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        xioctl(fd, VIDIOC_QBUF, &buf);
    }

    // Start stream
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(fd, VIDIOC_STREAMON, &type);

    tjhandle decompressor = tjInitDecompress();

    usb_state.fd = fd;
    usb_state.decompressor = decompressor;
    usb_state.num_buffers = num_buffers;
    usb_state.initialized = true;
    for (int i = 0; i < num_buffers; i++)
    {
        usb_state.buffers[i] = buffers[i];
    }
}

bool usb_recvFrame(unsigned char *destination)
{
    if (!usb_state.initialized)
    {
        printf("Need to initialize usb camera!\n");
        return false;
    }

    int fd = usb_state.fd;
    tjhandle decompressor = usb_state.decompressor;
    usb_Buffer *buffers = usb_state.buffers;
    int num_buffers = usb_state.num_buffers;

    // The device will now output data continuously.
    // We will use the FD_ZERO/FD_SET/select mechanisms
    // to wait until there is data available from the
    // device. We can specify how long we should wait,
    // and timeout if we think too much time has passed.
    fd_set fds;
    int r = 0;
    do
    {
        FD_ZERO(&fds);
        FD_SET(fd, &fds);
        timeval tv;
        tv.tv_sec = 2;
        tv.tv_usec = 0;
        r = select(fd + 1, &fds, NULL, NULL, &tv);
    } while ((r == -1 && (errno = EINTR)));

    if (r == -1)
    {
        printf("select failed in a bad way\n");
        return false;
    }

    // Data has arrived! Let's "dequeue" a buffer to get its data
    v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    xioctl(fd, VIDIOC_DQBUF, &buf);

    // Now we have gotten the data into one of our buffers.
    // buf.index                -> Which mmap'ed buffer is the data located in
    // buffers[buf.index].start -> Where in memory is the data located
    // buf.bytesused            -> Size of data chunk in bytes

    // Do whatever you want with the stream data here!
    // -----------------------------------------------
    unsigned char *jpg_data = (unsigned char*)buffers[buf.index].start;
    int jpg_size = buf.bytesused;

    TIMING("jpeg");

    // Decompress the image and store into the input gray buffer
    int jpg_subsamples, width, height;
    tjDecompressHeader2(decompressor,
        jpg_data,
        jpg_size,
        &width,
        &height,
        &jpg_subsamples);

    tjDecompress2(decompressor,
        jpg_data,
        jpg_size,
        destination,
        width,
        0,
        height,
        TJPF_RGB,
        TJFLAG_FASTDCT);

    TIMING("jpeg");

    // Queue buffer for writing again
    xioctl(fd, VIDIOC_QBUF, &buf);

    return true;
}

void usb_shutdown()
{
    int fd = usb_state.fd;
    tjhandle decompressor = usb_state.decompressor;
    usb_Buffer *buffers = usb_state.buffers;
    int num_buffers = usb_state.num_buffers;

    tjDestroy(decompressor);

    // Turn off stream
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(fd, VIDIOC_STREAMOFF, &type);

    // Unmap buffers
    for (int i = 0; i < num_buffers; ++i)
        munmap(buffers[i].start, buffers[i].length);

    // Tell the device that it can release memory
    v4l2_requestbuffers request = {};
    request.count = 0;
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = V4L2_MEMORY_MMAP;
    xioctl(fd, VIDIOC_REQBUFS, &request);

    v4l2_close(fd);
    printf("Closed device\n");
}
