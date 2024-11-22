/**
 * GoPro: video capture interface for GoPro with a video capture card.
 *
 * Author:
 *      Yifan Hou <yifanhou@stanford.edu>
 */

#ifndef _GOPRO_HEADER_
#define _GOPRO_HEADER_

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <RobotUtilities/timer_linux.h>
#include "hardware_interfaces/camera_interfaces.h"

namespace gopro {  // Add namespace

class GoPro : public CameraInterfaces {
public:
    struct GoProConfig {
        std::string device_name;
        double frame_width{1280};
        double frame_height{720};
        std::vector<int> crop_rows{-1, -1};
        std::vector<int> crop_cols{-1, -1};
        int fps{30};

        bool deserialize(const YAML::Node& node) {
            try {
                device_name = node["device_name"].as<std::string>();
                frame_width = node["frame_width"].as<double>();
                frame_height = node["frame_height"].as<double>();
                crop_rows = node["crop_rows"].as<std::vector<int>>();
                crop_cols = node["crop_cols"].as<std::vector<int>>();
                fps = node["fps"].as<int>();
            } catch (const std::exception& e) {
                std::cerr << "Failed to load the config file: " << e.what()
                          << std::endl;
                return false;
            }
            return true;
        }
    };

    GoPro();
    ~GoPro();

    bool init(RUT::TimePoint time0, const GoProConfig& config);
    cv::Mat next_rgb_frame_blocking() override;

private:
    struct Implementation;
    std::unique_ptr<Implementation> m_impl;
};

}  // namespace gopro

#endif