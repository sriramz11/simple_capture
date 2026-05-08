#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

struct Buffer {
    void *start = nullptr;
    size_t length = 0;
};

static int xioctl(int fd, unsigned long request, void *arg)
{
    int ret;
    do {
        ret = ioctl(fd, request, arg);
    } while (ret == -1 && errno == EINTR);

    return ret;
}

static uint32_t fourcc_from_string(const std::string &s)
{
    if (s.size() != 4) {
        return 0;
    }

    return v4l2_fourcc(s[0], s[1], s[2], s[3]);
}

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
    std::string device = argc > 1 ? argv[1] : "/dev/video0";
    std::string folder = argc > 2 ? argv[2] : "./csi_frames";
    int width = argc > 3 ? std::stoi(argv[3]) : 640;
    int height = argc > 4 ? std::stoi(argv[4]) : 480;
    std::string fmt_str = argc > 5 ? argv[5] : "NV12";
    int frame_count = argc > 6 ? std::stoi(argv[6]) : 10;

    uint32_t pixfmt = fourcc_from_string(fmt_str);

    if (pixfmt == 0 || frame_count <= 0) {
        std::cerr << "Usage: " << argv[0]
                  << " /dev/videoX output_folder width height FOURCC frame_count\n";
        std::cerr << "Example: " << argv[0]
                  << " /dev/video0 ./csi_frames 640 480 NV12 10\n";
        return 1;
    }

    int fd = open(device.c_str(), O_RDWR);
    if (fd < 0) {
        std::cerr << "open failed: " << strerror(errno) << "\n";
        return 1;
    }

    v4l2_capability cap {};
    if (xioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
        std::cerr << "VIDIOC_QUERYCAP failed: " << strerror(errno) << "\n";
        close(fd);
        return 1;
    }

    std::cout << "Driver: " << cap.driver << "\n";
    std::cout << "Card:   " << cap.card << "\n";

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) &&
        !(cap.device_caps & V4L2_CAP_VIDEO_CAPTURE)) {
        std::cerr << "This node does not look like a single-planar capture device.\n";
        std::cerr << "Try another /dev/videoX node from v4l2-ctl --list-devices.\n";
        close(fd);
        return 1;
    }

    v4l2_format fmt {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = pixfmt;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
        std::cerr << "VIDIOC_S_FMT failed: " << strerror(errno) << "\n";
        close(fd);
        return 1;
    }

    std::cout << "Format set: "
              << fmt.fmt.pix.width << "x" << fmt.fmt.pix.height
              << " " << fmt_str
              << " sizeimage=" << fmt.fmt.pix.sizeimage << "\n";

    v4l2_requestbuffers req {};
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        std::cerr << "VIDIOC_REQBUFS failed: " << strerror(errno) << "\n";
        close(fd);
        return 1;
    }

    std::vector<Buffer> buffers(req.count);

    for (unsigned int i = 0; i < req.count; i++) {
        v4l2_buffer buf {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (xioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
            std::cerr << "VIDIOC_QUERYBUF failed: " << strerror(errno) << "\n";
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
            std::cerr << "mmap failed: " << strerror(errno) << "\n";
            close(fd);
            return 1;
        }

        if (xioctl(fd, VIDIOC_QBUF, &buf) < 0) {
            std::cerr << "VIDIOC_QBUF failed: " << strerror(errno) << "\n";
            close(fd);
            return 1;
        }
    }

    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        std::cerr << "VIDIOC_STREAMON failed: " << strerror(errno) << "\n";
        close(fd);
        return 1;
    }

    for (int frame = 0; frame < frame_count; frame++) {
        pollfd pfd {};
        pfd.fd = fd;
        pfd.events = POLLIN;

        int pret = poll(&pfd, 1, 2000);
        if (pret <= 0) {
            std::cerr << "poll timeout or error\n";
            break;
        }

        v4l2_buffer buf {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (xioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
            std::cerr << "VIDIOC_DQBUF failed: " << strerror(errno) << "\n";
            break;
        }

        std::string filename = make_filename(folder, frame);
        std::ofstream out(filename, std::ios::binary);

        if (!out) {
            std::cerr << "Could not open output file: " << filename << "\n";
            break;
        }

        out.write(static_cast<char *>(buffers[buf.index].start),
                  static_cast<std::streamsize>(buf.bytesused));

        std::cout << "Saved " << filename
                  << " bytes=" << buf.bytesused
                  << " buffer_index=" << buf.index << "\n";

        if (xioctl(fd, VIDIOC_QBUF, &buf) < 0) {
            std::cerr << "VIDIOC_QBUF requeue failed: " << strerror(errno) << "\n";
            break;
        }
    }

    xioctl(fd, VIDIOC_STREAMOFF, &type);

    for (auto &b : buffers) {
        if (b.start && b.start != MAP_FAILED) {
            munmap(b.start, b.length);
        }
    }

    close(fd);
    return 0;
}
