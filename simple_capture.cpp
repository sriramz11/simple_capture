#include <opencv2/opencv.hpp>

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#include <iostream>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <cstring>

namespace fs = std::filesystem;

#define CAMERA_DEVICE "/dev/video0"
#define I2C_DEVICE    "/dev/i2c-8"
#define IMU_ADDRESS   0x68
#define CHIP_ID_REG   0x00
#define EXPECTED_ID   0x24

std::string get_time()
{
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);

    std::tm tm_buf {};
    localtime_r(&t, &tm_buf);

    std::ostringstream ss;
    ss << std::put_time(&tm_buf, "%Y-%m-%dT%H:%M:%S");
    return ss.str();
}

bool read_imu_chip_id(unsigned char& chip_id)
{
    int fd = open(I2C_DEVICE, O_RDWR);

    if (fd < 0)
    {
        std::cout << "Failed to open I2C device\n";
        return false;
    }

    if (ioctl(fd, I2C_SLAVE, IMU_ADDRESS) < 0)
    {
        std::cout << "Failed to select IMU address\n";
        close(fd);
        return false;
    }

    unsigned char reg = CHIP_ID_REG;

    if (write(fd, &reg, 1) != 1)
    {
        std::cout << "Failed to write chip ID register\n";
        close(fd);
        return false;
    }

    if (read(fd, &chip_id, 1) != 1)
    {
        std::cout << "Failed to read chip ID\n";
        close(fd);
        return false;
    }

    close(fd);
    return true;
}

double calculate_motion(cv::Mat& frame, cv::Mat& previous_gray)
{
    cv::Mat gray;
    cv::Mat diff;
    cv::Mat mask;

    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);

    if (previous_gray.empty())
    {
        previous_gray = gray.clone();
        return 0.0;
    }

    cv::absdiff(previous_gray, gray, diff);
    cv::threshold(diff, mask, 25, 255, cv::THRESH_BINARY);

    double changed_pixels = cv::countNonZero(mask);
    double total_pixels = mask.total();

    previous_gray = gray.clone();

    return changed_pixels / total_pixels;
}

void write_event_files(cv::Mat& frame,
                       double motion_score,
                       double accel_mag,
                       unsigned char chip_id,
                       bool imu_ok,
                       std::string trigger)
{
    fs::create_directories("events/event_001");

    cv::imwrite("events/event_001/frame.jpg", frame);

    std::ofstream csv("events/event_001/sensor.csv");
    csv << "timestamp,motion_score,accel_mag_g,imu_source\n";
    csv << get_time() << ","
        << motion_score << ","
        << accel_mag << ","
        << "simulated_after_chip_id_check\n";
    csv.close();

    std::ofstream json("events/event_001/event.json");
    json << "{\n";
    json << "  \"timestamp\": \"" << get_time() << "\",\n";
    json << "  \"trigger\": \"" << trigger << "\",\n";
    json << "  \"confidence\": 0.85,\n";

    json << "  \"camera\": {\n";
    json << "    \"device\": \"/dev/video0\",\n";
    json << "    \"file\": \"frame.jpg\"\n";
    json << "  },\n";

    json << "  \"imu\": {\n";
    json << "    \"bus\": \"/dev/i2c-8\",\n";
    json << "    \"address\": \"0x68\",\n";

    if (imu_ok)
    {
        json << "    \"chip_id\": \"0x"
             << std::hex
             << static_cast<int>(chip_id)
             << "\",\n";
    }
    else
    {
        json << "    \"chip_id\": \"unavailable\",\n";
    }

    json << "    \"accel_source\": \"simulated\"\n";
    json << "  },\n";

    json << "  \"files\": [\"frame.jpg\", \"sensor.csv\"]\n";
    json << "}\n";
    json.close();

    std::ofstream log("events/event_001/debug.log");
    log << "Event created at " << get_time() << "\n";
    log << "Trigger: " << trigger << "\n";
    log << "Motion score: " << motion_score << "\n";
    log << "Acceleration magnitude: " << accel_mag << "\n";
    log << "IMU verified: " << (imu_ok ? "yes" : "no") << "\n";
    log.close();
}

int main(int argc, char* argv[])
{
    bool manual_trigger = false;

    if (argc > 1)
    {
        std::string arg = argv[1];

        if (arg == "--manual")
        {
            manual_trigger = true;
        }
    }

    std::cout << "Starting simple dashcam prototype\n";

    unsigned char chip_id = 0;
    bool imu_ok = read_imu_chip_id(chip_id);

    if (imu_ok && chip_id == EXPECTED_ID)
    {
        std::cout << "BMI270 verified. Chip ID = 0x24\n";
    }
    else
    {
        std::cout << "IMU chip ID check failed or unexpected\n";
    }

    cv::VideoCapture camera(CAMERA_DEVICE, cv::CAP_V4L2);

    if (!camera.isOpened())
    {
        std::cout << "Failed to open camera\n";
        return 1;
    }

    camera.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    camera.set(cv::CAP_PROP_FPS, 30);

    cv::Mat frame;
    cv::Mat previous_gray;

    const double MOTION_THRESHOLD = 0.02;
    const double ACCEL_THRESHOLD = 1.70;

    for (int frame_count = 0; frame_count < 200; frame_count++)
    {
        camera >> frame;

        if (frame.empty())
        {
            std::cout << "Empty camera frame\n";
            continue;
        }

        double motion_score = calculate_motion(frame, previous_gray);

        double ax = 0.02;
        double ay = 0.01;
        double az = 1.00;

        if (frame_count == 80)
        {
            ax = 2.20;
            ay = 0.10;
            az = 1.00;
        }

        double accel_mag = std::sqrt((ax * ax) + (ay * ay) + (az * az));

        bool motion_event = motion_score > MOTION_THRESHOLD;
        bool imu_event = accel_mag > ACCEL_THRESHOLD;
        bool manual_event = manual_trigger && frame_count == 20;

        std::cout << "frame=" << frame_count
                  << " motion=" << motion_score
                  << " accel=" << accel_mag
                  << "\n";

        if (motion_event || imu_event || manual_event)
        {
            std::string trigger;

            if (manual_event)
            {
                trigger = "manual_test_trigger";
            }
            else if (motion_event && imu_event)
            {
                trigger = "motion_and_imu_detected";
            }
            else if (motion_event)
            {
                trigger = "motion_detected";
            }
            else
            {
                trigger = "simulated_sudden_acceleration";
            }

            write_event_files(frame,
                              motion_score,
                              accel_mag,
                              chip_id,
                              imu_ok,
                              trigger);

            std::cout << "Event saved in events/event_001\n";
            return 0;
        }
    }

    std::cout << "No event detected\n";
    std::cout << "Try running: ./dashcam --manual\n";

    return 0;
}
