#include <opencv2/opencv.hpp>

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <deque>
#include <cmath>
#include <cstring>
#include <cerrno>

namespace fs = std::filesystem;

static constexpr int BMI270_ADDR = 0x68;
static constexpr uint8_t BMI270_CHIP_ID_REG = 0x00;
static constexpr uint8_t BMI270_EXPECTED_CHIP_ID = 0x24;

struct SensorSample
{
    std::string timestamp;
    double ax_g;
    double ay_g;
    double az_g;
    double accel_mag_g;
    double motion_score;
    std::string source;
};

std::string now_iso_time()
{
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);

    std::tm tm_buf {};
    localtime_r(&t, &tm_buf);

    std::ostringstream ss;
    ss << std::put_time(&tm_buf, "%Y-%m-%dT%H:%M:%S");
    return ss.str();
}

std::string hex_byte(uint8_t value)
{
    std::ostringstream ss;
    ss << "0x"
       << std::hex
       << std::setw(2)
       << std::setfill('0')
       << static_cast<int>(value);
    return ss.str();
}

bool read_bmi270_chip_id(const std::string& i2c_dev, uint8_t& chip_id, std::string& error)
{
    int fd = open(i2c_dev.c_str(), O_RDWR);
    if (fd < 0)
    {
        error = "open failed: " + std::string(strerror(errno));
        return false;
    }

    if (ioctl(fd, I2C_SLAVE, BMI270_ADDR) < 0)
    {
        error = "ioctl I2C_SLAVE failed: " + std::string(strerror(errno));
        close(fd);
        return false;
    }

    uint8_t reg = BMI270_CHIP_ID_REG;

    if (::write(fd, &reg, 1) != 1)
    {
        error = "write chip-id register failed: " + std::string(strerror(errno));
        close(fd);
        return false;
    }

    if (::read(fd, &chip_id, 1) != 1)
    {
        error = "read chip-id value failed: " + std::string(strerror(errno));
        close(fd);
        return false;
    }

    close(fd);
    return true;
}

SensorSample make_simulated_imu_sample(int frame_count, double motion_score)
{
    SensorSample s;
    s.timestamp = now_iso_time();

    s.ax_g = 0.02 * std::sin(frame_count * 0.15);
    s.ay_g = 0.02 * std::cos(frame_count * 0.10);
    s.az_g = 1.00;
    s.source = "simulated";

    /*
     * Simulate one sudden acceleration event.
     * This makes testing deterministic during the interview.
     */
    if (frame_count == 80)
    {
        s.ax_g = 2.20;
        s.ay_g = 0.15;
        s.az_g = 1.10;
    }

    s.accel_mag_g = std::sqrt((s.ax_g * s.ax_g) +
                              (s.ay_g * s.ay_g) +
                              (s.az_g * s.az_g));

    s.motion_score = motion_score;
    return s;
}

double calculate_motion_score(const cv::Mat& frame, cv::Mat& previous_gray)
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

    double changed_pixels = static_cast<double>(cv::countNonZero(mask));
    double total_pixels = static_cast<double>(mask.total());

    previous_gray = gray.clone();

    if (total_pixels <= 0.0)
    {
        return 0.0;
    }

    return changed_pixels / total_pixels;
}

std::string event_dir_name(int event_id)
{
    std::ostringstream ss;
    ss << "event_"
       << std::setw(3)
       << std::setfill('0')
       << event_id;

    return ss.str();
}

int next_event_id(const fs::path& root)
{
    fs::create_directories(root);

    int id = 1;
    while (fs::exists(root / event_dir_name(id)))
    {
        id++;
    }

    return id;
}

void write_sensor_csv(const fs::path& path, const std::deque<SensorSample>& samples)
{
    std::ofstream csv(path);

    csv << "timestamp,ax_g,ay_g,az_g,accel_mag_g,motion_score,source\n";

    for (const auto& s : samples)
    {
        csv << s.timestamp << ","
            << s.ax_g << ","
            << s.ay_g << ","
            << s.az_g << ","
            << s.accel_mag_g << ","
            << s.motion_score << ","
            << s.source << "\n";
    }
}

void write_event_json(const fs::path& path,
                      const std::string& trigger,
                      double confidence,
                      const std::string& camera_dev,
                      const std::string& i2c_dev,
                      uint8_t chip_id,
                      bool chip_ok)
{
    std::ofstream json(path);

    json << "{\n";
    json << "  \"timestamp\": \"" << now_iso_time() << "\",\n";
    json << "  \"trigger\": \"" << trigger << "\",\n";
    json << "  \"confidence\": " << confidence << ",\n";
    json << "  \"camera\": {\n";
    json << "    \"device\": \"" << camera_dev << "\",\n";
    json << "    \"file\": \"frame.jpg\"\n";
    json << "  },\n";
    json << "  \"imu\": {\n";
    json << "    \"bus\": \"" << i2c_dev << "\",\n";
    json << "    \"address\": \"0x68\",\n";
    json << "    \"chip_id\": \"" << (chip_ok ? hex_byte(chip_id) : "unavailable") << "\",\n";
    json << "    \"accel_source\": \"simulated\"\n";
    json << "  },\n";
    json << "  \"files\": [\"frame.jpg\", \"sensor.csv\", \"debug.log\"]\n";
    json << "}\n";
}

void write_debug_log(const fs::path& path,
                     const std::string& trigger,
                     double motion_score,
                     double accel_mag,
                     bool chip_ok,
                     uint8_t chip_id,
                     const std::string& chip_error)
{
    std::ofstream log(path);

    log << "event_time=" << now_iso_time() << "\n";
    log << "trigger=" << trigger << "\n";
    log << "motion_score=" << motion_score << "\n";
    log << "accel_mag_g=" << accel_mag << "\n";

    if (chip_ok)
    {
        log << "bmi270_chip_id=" << hex_byte(chip_id) << "\n";
        log << "bmi270_expected_chip_id=" << hex_byte(BMI270_EXPECTED_CHIP_ID) << "\n";
        log << "imu_status=present\n";
    }
    else
    {
        log << "imu_status=failed\n";
        log << "imu_error=" << chip_error << "\n";
    }

    log << "camera_source=/dev/video0\n";
    log << "accel_source=simulated_after_chip_id_verification\n";
    log << "motion_trigger_threshold=0.020\n";
    log << "imu_trigger_threshold_g=1.700\n";
}

void save_event_package(const fs::path& root,
                        const cv::Mat& frame,
                        const std::deque<cv::Mat>& pre_frames,
                        const std::deque<SensorSample>& sensor_ring,
                        const std::string& trigger,
                        double confidence,
                        const std::string& camera_dev,
                        const std::string& i2c_dev,
                        uint8_t chip_id,
                        bool chip_ok,
                        const std::string& chip_error,
                        double motion_score,
                        double accel_mag)
{
    int id = next_event_id(root);
    fs::path event_dir = root / event_dir_name(id);

    fs::create_directories(event_dir);
    fs::create_directories(event_dir / "pre_event_frames");

    cv::imwrite((event_dir / "frame.jpg").string(), frame);

    int index = 0;
    for (const auto& f : pre_frames)
    {
        std::ostringstream filename;
        filename << "pre_" << std::setw(2) << std::setfill('0') << index << ".jpg";
        cv::imwrite((event_dir / "pre_event_frames" / filename.str()).string(), f);
        index++;
    }

    write_sensor_csv(event_dir / "sensor.csv", sensor_ring);

    write_event_json(event_dir / "event.json",
                     trigger,
                     confidence,
                     camera_dev,
                     i2c_dev,
                     chip_id,
                     chip_ok);

    write_debug_log(event_dir / "debug.log",
                    trigger,
                    motion_score,
                    accel_mag,
                    chip_ok,
                    chip_id,
                    chip_error);

    std::cout << "Event saved: " << event_dir << "\n";
}

int main(int argc, char* argv[])
{
    std::string camera_dev = "/dev/video0";
    std::string i2c_dev = "/dev/i2c-8";
    std::string output_root = "./events";

    bool manual_trigger = false;
    int max_frames = 300;

    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];

        if (arg == "--manual")
        {
            manual_trigger = true;
        }
        else if (arg == "--out" && i + 1 < argc)
        {
            output_root = argv[++i];
        }
        else if (arg == "--frames" && i + 1 < argc)
        {
            max_frames = std::stoi(argv[++i]);
        }
    }

    std::cout << "Dashcam prototype starting\n";
    std::cout << "Camera: " << camera_dev << "\n";
    std::cout << "IMU: " << i2c_dev << " addr 0x68\n";
    std::cout << "Output root: " << output_root << "\n";

    uint8_t chip_id = 0;
    std::string chip_error;
    bool chip_ok = read_bmi270_chip_id(i2c_dev, chip_id, chip_error);

    if (chip_ok)
    {
        std::cout << "BMI270 chip ID read: " << hex_byte(chip_id) << "\n";

        if (chip_id != BMI270_EXPECTED_CHIP_ID)
        {
            std::cout << "Warning: expected BMI270 chip ID 0x24\n";
        }
    }
    else
    {
        std::cout << "Warning: could not read BMI270 chip ID: "
                  << chip_error << "\n";
    }

    cv::VideoCapture cap;

    if (!cap.open(camera_dev, cv::CAP_V4L2))
    {
        std::cerr << "Failed to open camera at " << camera_dev << "\n";
        return 1;
    }

    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);

    if (!cap.isOpened())
    {
        std::cerr << "Camera is not opened\n";
        return 1;
    }

    cv::Mat previous_gray;
    std::deque<cv::Mat> pre_event_frames;
    std::deque<SensorSample> sensor_ring;

    constexpr size_t PRE_EVENT_FRAME_COUNT = 10;
    constexpr size_t SENSOR_RING_COUNT = 50;

    constexpr double MOTION_THRESHOLD = 0.020;
    constexpr double IMU_ACCEL_THRESHOLD_G = 1.700;

    for (int frame_count = 0; frame_count < max_frames; frame_count++)
    {
        cv::Mat frame;
        cap >> frame;

        if (frame.empty())
        {
            std::cerr << "Empty frame received\n";
            continue;
        }

        double motion_score = calculate_motion_score(frame, previous_gray);
        SensorSample sample = make_simulated_imu_sample(frame_count, motion_score);

        sensor_ring.push_back(sample);
        if (sensor_ring.size() > SENSOR_RING_COUNT)
        {
            sensor_ring.pop_front();
        }

        pre_event_frames.push_back(frame.clone());
        if (pre_event_frames.size() > PRE_EVENT_FRAME_COUNT)
        {
            pre_event_frames.pop_front();
        }

        bool motion_event = motion_score > MOTION_THRESHOLD;
        bool imu_event = sample.accel_mag_g > IMU_ACCEL_THRESHOLD_G;
        bool manual_event = manual_trigger && frame_count == 20;

        bool triggered = motion_event || imu_event || manual_event;

        std::cout << "frame=" << frame_count
                  << " motion=" << motion_score
                  << " accel_mag_g=" << sample.accel_mag_g
                  << "\n";

        if (triggered)
        {
            std::string trigger_name;
            double confidence = 0.80;

            if (motion_event && imu_event)
            {
                trigger_name = "motion_and_imu_detected";
                confidence = 0.95;
            }
            else if (motion_event)
            {
                trigger_name = "motion_detected";
                confidence = std::min(0.99, 0.50 + motion_score * 10.0);
            }
            else if (imu_event)
            {
                trigger_name = "simulated_sudden_acceleration";
                confidence = 0.85;
            }
            else
            {
                trigger_name = "manual_test_trigger";
                confidence = 1.00;
            }

            save_event_package(fs::path(output_root),
                               frame,
                               pre_event_frames,
                               sensor_ring,
                               trigger_name,
                               confidence,
                               camera_dev,
                               i2c_dev,
                               chip_id,
                               chip_ok,
                               chip_error,
                               motion_score,
                               sample.accel_mag_g);

            std::cout << "Prototype complete. One event package generated.\n";
            return 0;
        }
    }

    std::cout << "No event triggered before frame limit.\n";
    std::cout << "Try running with --manual for deterministic testing.\n";

    return 0;
}
