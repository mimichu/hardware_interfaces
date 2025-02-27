#pragma once

// Standard library includes
#include <unistd.h>
#include <csignal>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

// Third-party includes
#include <yaml-cpp/yaml.h>
#include <unsupported/Eigen/CXX11/Tensor>

// Robot utilities
#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>
#include <RobotUtilities/data_buffer.h>

// Hardware interfaces
#include <hardware_interfaces/robot_interfaces.h>
#include <hardware_interfaces/camera_interfaces.h>
#include <hardware_interfaces/ft_interfaces.h>
#include <hardware_interfaces/types.h>

// Force control
#include <force_control/admittance_controller.h>
#include <force_control/config_deserialize.h>

// Common hardware interfaces
#include <ati_netft/ati_netft.h>
#include <ur_rtde/ur_rtde.h>

// Platform-specific includes
#ifndef MACOS
#include <opencv2/opencv.hpp>
#include <gopro/gopro.h>
#include <realsense/realsense.h>
#include <robotiq_ft_modbus/robotiq_ft_modbus.h>
#endif

struct ManipServerConfig {
  std::string data_folder{""};
  bool run_robot_thread{false};
  bool run_wrench_thread{false};
  bool run_rgb_thread{false};
  bool plot_rgb{false};
  int rgb_buffer_size{5};
  int pose_buffer_size{100};
  int wrench_buffer_size{100};
  bool mock_hardware{false};
  bool bimanual{false};
  CameraSelection camera_selection{CameraSelection::NONE};
  ForceSensingMode force_sensing_mode{ForceSensingMode::NONE};
  RUT::Matrix6d low_damping{};
  std::vector<int> output_rgb_hw{};

  bool deserialize(const YAML::Node& node) {
    try {
      data_folder = node["data_folder"].as<std::string>();
      run_robot_thread = node["run_robot_thread"].as<bool>();
      run_wrench_thread = node["run_wrench_thread"].as<bool>();
      run_rgb_thread = node["run_rgb_thread"].as<bool>();
      plot_rgb = node["plot_rgb"].as<bool>();
      rgb_buffer_size = node["rgb_buffer_size"].as<int>();
      pose_buffer_size = node["pose_buffer_size"].as<int>();
      wrench_buffer_size = node["wrench_buffer_size"].as<int>();
      mock_hardware = node["mock_hardware"].as<bool>();
      bimanual = node["bimanual"].as<bool>();
      camera_selection = string_to_enum<CameraSelection>(
          node["camera_selection"].as<std::string>());
      force_sensing_mode = string_to_enum<ForceSensingMode>(
          node["force_sensing_mode"].as<std::string>());

      low_damping = RUT::deserialize_vector<RUT::Vector6d>(node["low_damping"])
                        .asDiagonal();
      output_rgb_hw = node["output_rgb_hw"].as<std::vector<int>>();

    } catch (const std::exception& e) {
      std::cerr << "Failed to load the config file: " << e.what() << std::endl;
      return false;
    }
    return true;
  }
};

/// @brief ManipServer class
/// This class is the main interface to the hardware. It initializes the hardware
/// interfaces, starts the threads to collect data, and provides the most recent
/// data points to the user.
/// Usage:
///   ManipServer server;
///   server.initialize("config.yaml");
///   // wait until the server is ready
///   while (!server.is_ready()) {
///     usleep(1000);
///   }
///   while (server.is_running()) {
///     Eigen::MatrixXd camera_rgb = server.get_camera_rgb(5); // get the most recent 5 data points
///     Eigen::MatrixXd pose = server.get_pose(5);
///     Eigen::MatrixXd wrench = server.get_wrench(5);
///     // get the timestamps of the most recently fetched data points
///     Eigen::VectorXd camera_rgb_timestamps = server.get_camera_rgb_timestamps_ms();
///     Eigen::VectorXd pose_timestamps = server.get_pose_timestamps_ms();
///     Eigen::VectorXd wrench_timestamps = server.get_wrench_timestamps_ms();
///     // do something with the data
///   }
///   server.join_threads();
class ManipServer {
 public:
  ManipServer() {};
  ManipServer(const std::string&);
  ~ManipServer();

  bool initialize(const std::string& config_file);
  void join_threads();
  bool is_ready();  // check if all buffers are full
  bool is_running();
  bool is_bimanual() { return _config.bimanual; }

  // getters: get the most recent k data points in the buffer
  const Eigen::MatrixXd get_camera_rgb(int k, int camera_id = 0);
  const Eigen::MatrixXd get_wrench(int k, int sensor_id = 0);
  const Eigen::MatrixXd get_pose(int k, int robot_id = 0);

  // the following functions return the timestamps of
  //  the most recent getter call of the corresponding feedback
  //  So size is already know
  const Eigen::VectorXd get_camera_rgb_timestamps_ms(int id = 0);
  const Eigen::VectorXd get_wrench_timestamps_ms(int id = 0);
  const Eigen::VectorXd get_pose_timestamps_ms(int id = 0);

  double get_timestamp_now_ms();  // access the current hardware time

  void set_high_level_maintain_position();
  void set_high_level_free_jogging();

  void set_target_pose(const Eigen::Ref<RUT::Vector7d> pose,
                       double dt_in_future_ms = 1000, int robot_id = 0);
  void set_force_controlled_axis(const RUT::Matrix6d& Tr, int n_af,
                                 int robot_id = 0);
  void set_stiffness_matrix(const RUT::Matrix6d& stiffness, int robot_id = 0);

  void schedule_waypoints(const Eigen::MatrixXd& waypoints,
                          const Eigen::VectorXd& timepoints_ms,
                          int robot_id = 0);
  void schedule_stiffness(const Eigen::MatrixXd& stiffness,
                          const Eigen::VectorXd& timepoints_ms,
                          int robot_id = 0);

  void clear_cmd_buffer();

  // data logging
  void start_saving_data_for_a_new_episode();
  void stop_saving_data();
  bool is_saving_data();

 private:
  // config
  ManipServerConfig _config;

  // list of id
  std::vector<int> _id_list;

  // data buffer
  std::vector<RUT::DataBuffer<Eigen::MatrixXd>> _camera_rgb_buffers;
  std::vector<RUT::DataBuffer<Eigen::VectorXd>> _pose_buffers;
  std::vector<RUT::DataBuffer<Eigen::VectorXd>> _wrench_buffers;
  // action buffer
  std::vector<RUT::DataBuffer<Eigen::VectorXd>> _waypoints_buffers;
  std::vector<RUT::DataBuffer<Eigen::MatrixXd>> _stiffness_buffers;

  std::vector<RUT::DataBuffer<double>> _camera_rgb_timestamp_ms_buffers;
  std::vector<RUT::DataBuffer<double>> _pose_timestamp_ms_buffers;
  std::vector<RUT::DataBuffer<double>> _wrench_timestamp_ms_buffers;
  std::vector<RUT::DataBuffer<double>> _waypoints_timestamp_ms_buffers;
  std::vector<RUT::DataBuffer<double>> _stiffness_timestamp_ms_buffers;

  std::deque<std::mutex> _camera_rgb_buffer_mtxs;
  std::deque<std::mutex> _pose_buffer_mtxs;
  std::deque<std::mutex> _wrench_buffer_mtxs;
  std::deque<std::mutex> _waypoints_buffer_mtxs;
  std::deque<std::mutex> _stiffness_buffer_mtxs;

  // timing
  RUT::Timer _timer;

  // additional configs as local variables
  std::vector<RUT::Matrix6d> _stiffnesses_high{};
  std::vector<RUT::Matrix6d> _stiffnesses_low{};
  std::vector<RUT::Matrix6d> _dampings_high{};
  std::vector<RUT::Matrix6d> _dampings_low{};

  //  hardware interfaces
  std::vector<std::shared_ptr<CameraInterfaces>> camera_ptrs;
  std::vector<std::shared_ptr<FTInterfaces>> force_sensor_ptrs;
  std::vector<std::shared_ptr<RobotInterfaces>> robot_ptrs;
  // controllers
  std::vector<AdmittanceController> _controllers;
  std::deque<std::mutex> _controller_mtxs;

  // threads
  std::vector<std::thread> _robot_threads;
  std::vector<std::thread> _wrench_threads;
  std::vector<std::thread> _rgb_threads;
  std::thread _rgb_plot_thread;

  // control variables to control the threads
  std::vector<std::string> _ctrl_rgb_folders;
  std::vector<std::ofstream> _ctrl_robot_data_streams;
  std::vector<std::ofstream> _ctrl_wrench_data_streams;
  bool _ctrl_flag_running = false;  // flag to terminate the program
  bool _ctrl_flag_saving = false;   // flag for ongoing data collection
  std::mutex _ctrl_mtx;

  // state variable indicating the status of the threads
  std::vector<bool> _states_robot_thread_ready{};
  std::vector<bool> _states_rgb_thread_ready{};
  std::vector<bool> _states_wrench_thread_ready{};
  bool _state_plot_thread_ready{false};

  std::vector<bool> _states_robot_thread_saving{};
  std::vector<bool> _states_rgb_thread_saving{};
  std::vector<bool> _states_wrench_thread_saving{};

  std::vector<int> _states_robot_seq_id{};
  std::vector<int> _states_rgb_seq_id{};
  std::vector<int> _states_wrench_seq_id{};

  // shared variables between camera thread and plot thread
  std::vector<cv::Mat> _color_mats;
  std::deque<std::mutex> _color_mat_mtxs;

  // shared variables between robot thread and wrench thread
  std::vector<Eigen::VectorXd> _poses_fb;
  std::deque<std::mutex> _poses_fb_mtxs;

  // temp variables storing timestamps of data just being fetched
  std::vector<Eigen::VectorXd> _camera_rgb_timestamps_ms;
  std::vector<Eigen::VectorXd> _pose_timestamps_ms;
  std::vector<Eigen::VectorXd> _wrench_timestamps_ms;

  void robot_loop(const RUT::TimePoint& time0, int robot_id);
  void rgb_loop(const RUT::TimePoint& time0, int camera_id);
  void wrench_loop(const RUT::TimePoint& time0, int publish_rate,
                   int sensor_id);
  void rgb_plot_loop();  // opencv plotting does not support multi-threading
};