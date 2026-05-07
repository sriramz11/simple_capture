/*
 * simple_csi_capture.cpp
 *
 * Purpose:
 *   Capture raw frames from a Linux V4L2 camera device and save them
 *   into a folder as frame_000.raw, frame_001.raw, ...
 *
 * Target use:
 *   Radxa ROCK 4D / RK3576 / Radxa OS / MIPI CSI camera
 *
 * API used:
 *   V4L2 = Video4Linux2
 *
 * Important idea:
 *   The CSI camera data is handled by the Linux kernel camera driver.
 *   Userspace sees the final camera stream as /dev/videoX.
 *
 * Capture method:
 *   V4L2 MMAP streaming.
 *
 * Why MMAP?
 *   Camera frames are large.
 *   Instead of copying frame data again and again, the kernel allocates
 *   video buffers, the camera/ISP/DMA writes frames into those buffers,
 *   and userspace maps those buffers using mmap().
 */

#include <fcntl.h>          // open()
#include <linux/videodev2.h> // V4L2 structs, constants, ioctl request codes
#include <poll.h>           // poll()
#include <sys/ioctl.h>      // ioctl()
#include <sys/mman.h>       // mmap(), munmap()
#include <unistd.h>         // close()

#include <cerrno>           // errno
#include <cstring>          // strerror()
#include <fstream>          // std::ofstream
#include <iomanip>          // std::setw, std::setfill
#include <iostream>         // std::cout, std::cerr
#include <sstream>          // std::ostringstream
#include <string>           // std::string
#include <vector>           // std::vector

/*
 * Buffer represents one MMAP buffer.
 *
 * start:
 *   Userspace pointer to the mapped kernel video buffer.
 *
 * length:
 *   Size of that buffer in bytes.
 */
struct Buffer {
    void *start = nullptr;
    size_t length = 0;
};

/*
 * xioctl()
 *
 * ioctl() can be interrupted by a signal.
 * If that happens, ioctl() returns -1 and errno becomes EINTR.
 *
 * This helper retries ioctl() automatically if EINTR happens.
 */
static int xioctl(int fd, unsigned long request, void *arg)
{
    int ret;

    do {
        ret = ioctl(fd, request, arg);
    } while (ret == -1 && errno == EINTR);

    return ret;
}

/*
 * Convert a 4-character string like "NV12" or "YUYV"
 * into a V4L2 pixel format code.
 *
 * V4L2 uses FOURCC codes for pixel formats.
 *
 * Examples:
 *   "NV12" -> V4L2_PIX_FMT_NV12
 *   "YUYV" -> V4L2_PIX_FMT_YUYV
 *   "MJPG" -> V4L2_PIX_FMT_MJPEG
 */
static uint32_t fourcc_from_string(const std::string &s)
{
    if (s.size() != 4) {
        return 0;
    }

    return v4l2_fourcc(s[0], s[1], s[2], s[3]);
}

/*
 * Create output filename.
 *
 * Example:
 *   folder = "./csi_frames"
 *   frame_num = 3
 *
 * Output:
 *   ./csi_frames/frame_003.raw
 */
static std::string make_filename(const std::string &folder, int frame_num)
{
    std::ostringstream oss;

    oss << folder << "/frame_"
        << std::setw(3) << std::setfill('0') << frame_num
        << ".raw";

    return oss.str();
}

int main(int argc, char **argv)
{
    /*
     * Command line arguments:
     *
     * argv[1] = video device path
     * argv[2] = output folder
     * argv[3] = width
     * argv[4] = height
     * argv[5] = pixel format
     * argv[6] = number of frames
     *
     * Example:
     *   ./simple_csi_capture /dev/video0 ./csi_frames 640 480 NV12 10
     */

    std::string device = argc > 1 ? argv[1] : "/dev/video0";
    std::string folder = argc > 2 ? argv[2] : "./csi_frames";
    int width = argc > 3 ? std::stoi(argv[3]) : 640;
    int height = argc > 4 ? std::stoi(argv[4]) : 480;
    std::string fmt_str = argc > 5 ? argv[5] : "NV12";
    int frame_count = argc > 6 ? std::stoi(argv[6]) : 10;

    /*
     * Convert user pixel format string into V4L2 FOURCC format.
     */
    uint32_t pixfmt = fourcc_from_string(fmt_str);

    if (pixfmt == 0 || frame_count <= 0) {
        std::cerr << "Usage:\n";
        std::cerr << "  " << argv[0]
                  << " /dev/videoX output_folder width height FOURCC frame_count\n\n";
        std::cerr << "Example:\n";
        std::cerr << "  " << argv[0]
                  << " /dev/video0 ./csi_frames 640 480 NV12 10\n";
        return 1;
    }

    /*
     * STEP 1:
     * Open the V4L2 camera device.
     *
     * /dev/video0, /dev/video1, etc. are device files exposed by Linux.
     * For a CSI camera, the kernel driver handles the hardware side.
     */
    int fd = open(device.c_str(), O_RDWR);

    if (fd < 0) {
        std::cerr << "open(" << device << ") failed: "
                  << strerror(errno) << "\n";
        return 1;
    }

    /*
     * STEP 2:
     * Query camera capability.
     *
     * This asks:
     *   What driver owns this node?
     *   Is this a capture device?
     *   Does it support streaming?
     */
    v4l2_capability cap {};

    if (xioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
        std::cerr << "VIDIOC_QUERYCAP failed: "
                  << strerror(errno) << "\n";
        close(fd);
        return 1;
    }

    std::cout << "Driver: " << cap.driver << "\n";
    std::cout << "Card:   " << cap.card << "\n";
    std::cout << "Bus:    " << cap.bus_info << "\n";

    /*
     * Some drivers report capability in cap.capabilities.
     * Some report device-specific capability in cap.device_caps.
     */
    uint32_t caps = cap.device_caps ? cap.device_caps : cap.capabilities;

    /*
     * This simple program supports single-planar capture:
     *   V4L2_BUF_TYPE_VIDEO_CAPTURE
     *
     * Some Rockchip camera nodes may use multi-planar capture:
     *   V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
     *
     * For simple interview/demo code, try another /dev/videoX node first
     * if this one is not single-planar.
     */
    if (!(caps & V4L2_CAP_VIDEO_CAPTURE)) {
        std::cerr << "This node is not a single-planar V4L2 capture device.\n";
        std::cerr << "Try another /dev/videoX node from:\n";
        std::cerr << "  v4l2-ctl --list-devices\n";
        std::cerr << "  grep -H '' /sys/class/video4linux/video*/name\n";
        close(fd);
        return 1;
    }

    if (!(caps & V4L2_CAP_STREAMING)) {
        std::cerr << "This device does not support V4L2 streaming I/O.\n";
        close(fd);
        return 1;
    }

    /*
     * STEP 3:
     * Set camera format.
     *
     * We request:
     *   width
     *   height
     *   pixel format such as NV12 or YUYV
     *
     * The driver may adjust the actual format.
     * That is why we print the returned values after VIDIOC_S_FMT.
     */
    v4l2_format fmt {};

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = pixfmt;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
        std::cerr << "VIDIOC_S_FMT failed for "
                  << width << "x" << height << " " << fmt_str
                  << ": " << strerror(errno) << "\n";

        std::cerr << "Check supported formats using:\n";
        std::cerr << "  v4l2-ctl -d " << device << " --list-formats-ext\n";

        close(fd);
        return 1;
    }

    std::cout << "Requested format: "
              << width << "x" << height << " " << fmt_str << "\n";

    std::cout << "Driver selected:  "
              << fmt.fmt.pix.width << "x"
              << fmt.fmt.pix.height
              << " sizeimage=" << fmt.fmt.pix.sizeimage
              << "\n";

    /*
     * STEP 4:
     * Request MMAP buffers from the driver.
     *
     * req.count = 4 means we ask for 4 video buffers.
     *
     * Why multiple buffers?
     *   While userspace is processing one frame, the camera can fill another.
     *   This reduces frame drops.
     */
    v4l2_requestbuffers req {};

    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        std::cerr << "VIDIOC_REQBUFS failed: "
                  << strerror(errno) << "\n";
        close(fd);
        return 1;
    }

    if (req.count < 2) {
        std::cerr << "Driver returned too few buffers: "
                  << req.count << "\n";
        close(fd);
        return 1;
    }

    std::cout << "Driver allocated " << req.count << " buffers\n";

    /*
     * Store all buffer mappings in this vector.
     */
    std::vector<Buffer> buffers(req.count);

    /*
     * STEP 5:
     * Query each buffer and mmap it into userspace.
     *
     * VIDIOC_QUERYBUF tells us:
     *   buffer size
     *   offset used for mmap()
     *
     * mmap() maps kernel video buffer memory into this process.
     */
    for (unsigned int i = 0; i < req.count; i++) {
        v4l2_buffer buf {};

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (xioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
            std::cerr << "VIDIOC_QUERYBUF failed for buffer "
                      << i << ": " << strerror(errno) << "\n";
            close(fd);
            return 1;
        }

        buffers[i].length = buf.length;

        buffers[i].start = mmap(nullptr,
                                buf.length,
                                PROT_READ | PROT_WRITE,
                                MAP_SHARED,
                                fd,
                                buf.m.offset);

        if (buffers[i].start == MAP_FAILED) {
            std::cerr << "mmap failed for buffer "
                      << i << ": " << strerror(errno) << "\n";
            close(fd);
            return 1;
        }

        /*
         * STEP 6:
         * Queue the empty buffer to the driver.
         *
         * After QBUF, the driver owns this buffer and can fill it
         * with camera frame data.
         */
        if (xioctl(fd, VIDIOC_QBUF, &buf) < 0) {
            std::cerr << "VIDIOC_QBUF failed for buffer "
                      << i << ": " << strerror(errno) << "\n";
            close(fd);
            return 1;
        }
    }

    /*
     * STEP 7:
     * Start streaming.
     *
     * After STREAMON, the camera pipeline starts producing frames.
     * Hardware/driver will fill the queued buffers.
     */
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        std::cerr << "VIDIOC_STREAMON failed: "
                  << strerror(errno) << "\n";
        close(fd);
        return 1;
    }

    std::cout << "Streaming started\n";

    /*
     * STEP 8:
     * Capture frames.
     *
     * Loop:
     *   poll() waits until a frame is ready.
     *   VIDIOC_DQBUF gets one filled buffer from the driver.
     *   Write the frame data to a file.
     *   VIDIOC_QBUF gives the buffer back to the driver.
     */
    for (int frame = 0; frame < frame_count; frame++) {
        /*
         * poll() waits for frame readiness.
         *
         * Timeout = 2000 ms.
         *
         * This prevents the program from hanging forever if the camera
         * stops producing frames.
         */
        pollfd pfd {};

        pfd.fd = fd;
        pfd.events = POLLIN;

        int pret = poll(&pfd, 1, 2000);

        if (pret < 0) {
            std::cerr << "poll failed: "
                      << strerror(errno) << "\n";
            break;
        }

        if (pret == 0) {
            std::cerr << "poll timeout: no frame received\n";
            break;
        }

        /*
         * Dequeue a filled buffer.
         *
         * After DQBUF, userspace owns the buffer temporarily.
         */
        v4l2_buffer buf {};

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (xioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
            std::cerr << "VIDIOC_DQBUF failed: "
                      << strerror(errno) << "\n";
            break;
        }

        /*
         * Create frame filename.
         */
        std::string filename = make_filename(folder, frame);

        /*
         * Save raw frame data.
         *
         * buf.bytesused tells how many valid bytes are in this buffer.
         */
        std::ofstream out(filename, std::ios::binary);

        if (!out) {
            std::cerr << "Could not open output file: "
                      << filename << "\n";

            /*
             * Requeue buffer before breaking.
             * Otherwise driver loses one available buffer.
             */
            xioctl(fd, VIDIOC_QBUF, &buf);
            break;
        }

        out.write(static_cast<char *>(buffers[buf.index].start),
                  static_cast<std::streamsize>(buf.bytesused));

        out.close();

        std::cout << "Saved " << filename
                  << " bytes=" << buf.bytesused
                  << " buffer_index=" << buf.index
                  << "\n";

        /*
         * Requeue the buffer.
         *
         * After QBUF, the driver owns it again and can fill it with
         * another frame.
         */
        if (xioctl(fd, VIDIOC_QBUF, &buf) < 0) {
            std::cerr << "VIDIOC_QBUF requeue failed: "
                      << strerror(errno) << "\n";
            break;
        }
    }

    /*
     * STEP 9:
     * Stop streaming.
     */
    if (xioctl(fd, VIDIOC_STREAMOFF, &type) < 0) {
        std::cerr << "VIDIOC_STREAMOFF failed: "
                  << strerror(errno) << "\n";
    } else {
        std::cout << "Streaming stopped\n";
    }

    /*
     * STEP 10:
     * Unmap all MMAP buffers.
     */
    for (auto &b : buffers) {
        if (b.start && b.start != MAP_FAILED) {
            munmap(b.start, b.length);
        }
    }

    /*
     * STEP 11:
     * Close the camera device.
     */
    close(fd);

    std::cout << "Done\n";

    return 0;
}