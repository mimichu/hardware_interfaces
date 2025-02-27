// #include "table_top_manip/manip_server.h"
// #include "helpers.hpp"

// #include "gopro/gopro.h"
// #include "realsense/Realsense.h"
// #include "robotiq_ft_modbus/robotiq_ft_modbus.h"
// // #include "path_to_realsense_header/Realsense.h"
// // #include "path_to_robotiq_header/RobotiqFTModbus.h"


// ManipServer::ManipServer(const std::string& config_path) {
//   initialize(config_path);
// }

// ManipServer::~ManipServer() {}

// bool ManipServer::initialize(const std::string& config_path) {
//   std::cout << "[ManipServer] Initializing.\n";

//   RUT::TimePoint time0 = _timer.tic();

//   // read config files
//   std::cout << "[ManipServer] Reading config files.\n";
//   YAML::Node config;
//   try {
//     config = YAML::LoadFile(config_path);
//     _config.deserialize(config);
//   } catch (const std::exception& e) {
//     std::cerr << "Failed to load the config file: " << e.what() << std::endl;
//     return false;
//   }

//   if (_config.bimanual) {
//     _id_list = {0, 1};
//   } else {
//     _id_list = {0};
//   }

//   std::cout << "_id_list: " << _id_list.size() << std::endl;

//   // parameters to be obtained from config
//   std::vector<int> wrench_publish_rate;

//   std::cout << "[ManipServer] bimanual: " << _config.bimanual << std::endl;
//   std::cout << "[ManipServer] Initialize each hardware interface.\n";

//   // initialize hardwares
//   if (!_config.mock_hardware) {
//     for (int id : _id_list) {
//       // Robot
//       // TODO: support other robots
//       URRTDE::URRTDEConfig robot_config;
//       try {
//         robot_config.deserialize(config["ur_rtde" + std::to_string(id)]);
//       } catch (const std::exception& e) {
//         std::cerr << "Failed to load the robot config file: " << e.what()
//                   << std::endl;
//         return false;
//       }
//       robot_ptrs.emplace_back(std::make_shared<URRTDE>());
//       auto* urrtde_ptr = static_cast<URRTDE*>(robot_ptrs[id].get());
//       if (!urrtde_ptr->init(time0, robot_config)) {
//         std::cerr << "Failed to initialize UR RTDE for id " << id
//                   << ". Exiting." << std::endl;
//         return false;
//       }

//       // Camera
//       if (_config.camera_selection == CameraSelection::GOPRO) {
//         gopro::GoPro::GoProConfig gopro_config;
//         try {
//           gopro_config.deserialize(config["gopro" + std::to_string(id)]);
//         } catch (const std::exception& e) {
//           std::cerr << "Failed to load the GoPro config file: " << e.what()
//                     << std::endl;
//           return false;
//         }

//         // camera_ptrs.emplace_back(new GoPro);
//         // GoPro* gopro_ptr = static_cast<GoPro*>(camera_ptrs[id].get());
//         camera_ptrs.emplace_back(std::make_shared<gopro::GoPro>());
//         auto* gopro_ptr = static_cast<gopro::GoPro*>(camera_ptrs[id].get()); //Changed
//         if (!gopro_ptr->init(time0, gopro_config)) {
//           std::cerr << "Failed to initialize GoPro for id " << id
//                     << ". Exiting." << std::endl;
//           return false;
//         }
//       } else if (_config.camera_selection == CameraSelection::REALSENSE) {
//         Realsense::RealsenseConfig realsense_config;
//         try {
//           realsense_config.deserialize(
//               config["realsense" + std::to_string(id)]);
//         } catch (const std::exception& e) {
//           std::cerr << "Failed to load the Realsense config file: " << e.what()
//                     << std::endl;
//           return false;
//         }
//         // camera_ptrs.emplace_back(new Realsense);
//         // Realsense* realsense_ptr =
//         //     static_cast<Realsense*>(camera_ptrs[id].get());
//         camera_ptrs.emplace_back(std::make_shared<Realsense>());
//         auto* realsense_ptr = static_cast<Realsense*>(camera_ptrs[id].get());

//         if (!realsense_ptr->init(time0, realsense_config)) {
//           std::cerr << "Failed to initialize realsense for id " << id
//                     << ". Exiting." << std::endl;
//           return false;
//         }
//       } else {
//         std::cerr << "Invalid camera selection. Exiting." << std::endl;
//         return false;
//       }

//       // force sensor
//       if (_config.force_sensing_mode == ForceSensingMode::FORCE_MODE_ATI) {
//         ATINetft::ATINetftConfig ati_config;
//         try {
//           ati_config.deserialize(config["ati_netft" + std::to_string(id)]);
//         } catch (const std::exception& e) {
//           std::cerr << "Failed to load the ATI Netft config file: " << e.what()
//                     << std::endl;
//           return false;
//         }
//         force_sensor_ptrs.emplace_back(new ATINetft);
//         ATINetft* ati_ptr = static_cast<ATINetft*>(force_sensor_ptrs[id].get());
//         if (!ati_ptr->init(time0, ati_config)) {
//           std::cerr << "Failed to initialize ATI Netft for id " << id
//                     << ". Exiting." << std::endl;
//           return false;
//         }
//         wrench_publish_rate.push_back(ati_config.publish_rate);
//       } else if (_config.force_sensing_mode ==
//                  ForceSensingMode::FORCE_MODE_ROBOTIQ) {
//         RobotiqFTModbus::RobotiqFTModbusConfig robotiq_config;
//         try {
//           robotiq_config.deserialize(
//               config["robotiq_ft_modbus" + std::to_string(id)]);
//         } catch (const std::exception& e) {
//           std::cerr << "Failed to load the Robotiq FT Modbus config file: "
//                     << e.what() << std::endl;
//           return false;
//         }
//         force_sensor_ptrs.emplace_back(new RobotiqFTModbus);
//         RobotiqFTModbus* robotiq_ptr =
//             static_cast<RobotiqFTModbus*>(force_sensor_ptrs[id].get());
//         if (!robotiq_ptr->init(time0, robotiq_config)) {
//           std::cerr << "Failed to initialize Robotiq FT Modbus for id " << id
//                     << ". Exiting." << std::endl;
//           return false;
//         }
//         wrench_publish_rate.push_back(robotiq_config.publish_rate);
//       } else {
//         std::cerr << "Invalid force sensing mode. Exiting." << std::endl;
//         return false;
//       }
//     }
//   } else {
//     // mock hardware
//     for (int id : _id_list) {
//       wrench_publish_rate.push_back(7000);
//     }
//   }

//   // initialize Admittance controller
//   for (int id : _id_list) {
//     AdmittanceController::AdmittanceControllerConfig admittance_config;
//     try {
//       deserialize(config["admittance_controller" + std::to_string(id)],
//                   admittance_config);
//     } catch (const std::exception& e) {
//       std::cerr << "Failed to load the admittance controller config file: "
//                 << e.what() << std::endl;
//       return false;
//     }

//     _controllers.emplace_back();
//     _controller_mtxs.emplace_back();
//     RUT::Vector7d pose = RUT::Vector7d::Zero();
//     if (!_config.mock_hardware) {
//       robot_ptrs[id]->getCartesian(pose);
//     }
//     if (!_controllers[id].init(time0, admittance_config, pose)) {
//       std::cerr << "Failed to initialize admittance controller for id " << id
//                 << ". Exiting." << std::endl;
//       return false;
//     }
//     RUT::Matrix6d Tr = RUT::Matrix6d::Identity();
//     // The robot should not behave with any compliance during initialization.
//     // The user needs to set the desired compliance afterwards.
//     int n_af = 0;
//     _controllers[id].setForceControlledAxis(Tr, n_af);

//     _stiffnesses_high.push_back(admittance_config.compliance6d.stiffness);
//     _stiffnesses_low.push_back(RUT::Matrix6d::Zero());
//     _dampings_high.push_back(admittance_config.compliance6d.damping);
//     _dampings_low.push_back(_config.low_damping);
//   }

//   // create the data buffers
//   std::cout << "[ManipServer] Creating data buffers.\n";
//   for (int id : _id_list) {
//     _camera_rgb_buffers.push_back(RUT::DataBuffer<Eigen::MatrixXd>());
//     _pose_buffers.push_back(RUT::DataBuffer<Eigen::VectorXd>());
//     _wrench_buffers.push_back(RUT::DataBuffer<Eigen::VectorXd>());
//     _waypoints_buffers.push_back(RUT::DataBuffer<Eigen::VectorXd>());
//     _stiffness_buffers.push_back(RUT::DataBuffer<Eigen::MatrixXd>());

//     _camera_rgb_timestamp_ms_buffers.push_back(RUT::DataBuffer<double>());
//     _pose_timestamp_ms_buffers.push_back(RUT::DataBuffer<double>());
//     _wrench_timestamp_ms_buffers.push_back(RUT::DataBuffer<double>());
//     _waypoints_timestamp_ms_buffers.push_back(RUT::DataBuffer<double>());
//     _stiffness_timestamp_ms_buffers.push_back(RUT::DataBuffer<double>());

//     _camera_rgb_buffers[id].initialize(
//         _config.rgb_buffer_size, 3 * _config.output_rgb_hw[0],
//         _config.output_rgb_hw[1], "camera_rgb" + std::to_string(id));

//     _pose_buffers[id].initialize(_config.pose_buffer_size, 7, 1,
//                                  "pose" + std::to_string(id));
//     _wrench_buffers[id].initialize(_config.wrench_buffer_size, 6, 1,
//                                    "wrench" + std::to_string(id));

//     _waypoints_buffers[id].initialize(-1, 7, 1,
//                                       "waypoints" + std::to_string(id));
//     _stiffness_buffers[id].initialize(-1, 6, 6,
//                                       "stiffness" + std::to_string(id));

//     _camera_rgb_timestamp_ms_buffers[id].initialize(
//         _config.rgb_buffer_size, 1, 1,
//         "camera_rgb" + std::to_string(id) + "_timestamp_ms");
//     _pose_timestamp_ms_buffers[id].initialize(
//         _config.pose_buffer_size, 1, 1,
//         "pose" + std::to_string(id) + "_timestamp_ms");
//     _wrench_timestamp_ms_buffers[id].initialize(
//         _config.wrench_buffer_size, 1, 1,
//         "wrench" + std::to_string(id) + "_timestamp_ms");
//     _waypoints_timestamp_ms_buffers[id].initialize(
//         -1, 1, 1, "waypoints" + std::to_string(id) + "_timestamp_ms");
//     _stiffness_timestamp_ms_buffers[id].initialize(
//         -1, 1, 1, "stiffness" + std::to_string(id) + "_timestamp_ms");
//   }

//   // initialize the buffer mutexes
//   for (int id : _id_list) {
//     _camera_rgb_buffer_mtxs.emplace_back();
//     _pose_buffer_mtxs.emplace_back();
//     _wrench_buffer_mtxs.emplace_back();
//     _waypoints_buffer_mtxs.emplace_back();
//     _stiffness_buffer_mtxs.emplace_back();
//   }

//   // initialize thread status variables
//   for (int id : _id_list) {
//     _states_robot_thread_ready.push_back(false);
//     _states_rgb_thread_ready.push_back(false);
//     _states_wrench_thread_ready.push_back(false);
//     _states_robot_thread_saving.push_back(false);
//     _states_rgb_thread_saving.push_back(false);
//     _states_wrench_thread_saving.push_back(false);
//     _states_robot_seq_id.push_back(0);
//     _states_rgb_seq_id.push_back(0);
//     _states_wrench_seq_id.push_back(0);
//   }

//   // initialize additional shared variables
//   for (int id : _id_list) {
//     _ctrl_rgb_folders.push_back("");
//     _ctrl_robot_data_streams.push_back(std::ofstream());
//     _ctrl_wrench_data_streams.push_back(std::ofstream());
//     _color_mats.push_back(cv::Mat());
//     _color_mat_mtxs.emplace_back();
//     _poses_fb.push_back(Eigen::VectorXd());
//     _poses_fb_mtxs.emplace_back();
//     _camera_rgb_timestamps_ms.push_back(Eigen::VectorXd());
//     _pose_timestamps_ms.push_back(Eigen::VectorXd());
//     _wrench_timestamps_ms.push_back(Eigen::VectorXd());
//   }

//   // kickoff the threads
//   _ctrl_flag_running = true;
//   std::cout << "[ManipServer] Starting the threads.\n";
//   for (int id : _id_list) {
//     if (_config.run_rgb_thread) {
//       _rgb_threads.emplace_back(&ManipServer::rgb_loop, this, std::ref(time0),
//                                 id);
//     }
//     if (_config.run_wrench_thread) {
//       _wrench_threads.emplace_back(&ManipServer::wrench_loop, this,
//                                    std::ref(time0), wrench_publish_rate[id],
//                                    id);
//     }
//     if (_config.run_robot_thread) {
//       _robot_threads.emplace_back(&ManipServer::robot_loop, this,
//                                   std::ref(time0), id);
//     }
//   }
//   if (_config.plot_rgb) {
//     // pause 1s, then start the rgb plot thread
//     std::this_thread::sleep_for(std::chrono::seconds(1));
//     _rgb_plot_thread = std::thread(&ManipServer::rgb_plot_loop, this);
//   }

//   // wait for threads to be ready
//   std::cout << "[ManipServer] Waiting for threads to be ready.\n";
//   while (true) {
//     bool all_ready = true;
//     {
//       std::lock_guard<std::mutex> lock(_ctrl_mtx);
//       for (int id : _id_list) {
//         if (_config.run_rgb_thread) {
//           all_ready = all_ready && _states_rgb_thread_ready[id];
//         }
//         if (_config.run_wrench_thread) {
//           all_ready = all_ready && _states_wrench_thread_ready[id];
//         }
//         if (_config.run_robot_thread) {
//           all_ready = all_ready && _states_robot_thread_ready[id];
//         }
//       }
//       if (_config.plot_rgb) {
//         all_ready = all_ready && _state_plot_thread_ready;
//       }
//     }
//     if (all_ready) {
//       break;
//     }
//     std::this_thread::sleep_for(std::chrono::milliseconds(200));
//   }
//   std::cout << "[ManipServer] All threads are ready." << std::endl;
//   std::cout << "[ManipServer] Done initialization." << std::endl;
//   return true;
// }

// void ManipServer::join_threads() {
//   std::cout << "[ManipServer]: Waiting for threads to join." << std::endl;
//   {
//     std::lock_guard<std::mutex> lock(_ctrl_mtx);
//     _ctrl_flag_running = false;
//   }

//   // join the threads
//   if (_config.run_rgb_thread) {
//     std::cout << "[ManipServer]: Waiting for rgb threads to join." << std::endl;
//     for (auto& rgb_thread : _rgb_threads) {
//       rgb_thread.join();
//     }
//   }
//   if (_config.run_wrench_thread) {
//     std::cout << "[ManipServer]: Waiting for wrench threads to join."
//               << std::endl;
//     for (auto& wrench_thread : _wrench_threads) {
//       wrench_thread.join();
//     }
//   }
//   if (_config.run_robot_thread) {
//     std::cout << "[ManipServer]: Waiting for robot threads to join."
//               << std::endl;
//     for (auto& robot_thread : _robot_threads) {
//       robot_thread.join();
//     }
//   }
//   if (_config.plot_rgb) {
//     std::cout << "[ManipServer]: Waiting for plotting thread to join."
//               << std::endl;
//     _rgb_plot_thread.join();
//   }

//   std::cout << "[ManipServer]: Threads have joined. Exiting." << std::endl;
// }

// bool ManipServer::is_ready() {
//   for (int id : _id_list) {
//     if (_config.run_rgb_thread) {
//       std::lock_guard<std::mutex> lock(_camera_rgb_buffer_mtxs[id]);
//       if (!_camera_rgb_buffers[id].is_full()) {
//         std::cout << id << ": Camera RGB buffer not full: size: "
//                   << _camera_rgb_buffers[id].size() << std::endl;
//         return false;
//       }
//     }

//     if (_config.run_robot_thread) {
//       std::lock_guard<std::mutex> lock(_pose_buffer_mtxs[id]);
//       if (!_pose_buffers[id].is_full()) {
//         std::cout << id << ": Pose buffer not full: size: "
//                   << _pose_buffers[id].size() << std::endl;
//         return false;
//       }
//     }

//     if (_config.run_wrench_thread) {
//       std::lock_guard<std::mutex> lock(_wrench_buffer_mtxs[id]);
//       if (!_wrench_buffers[id].is_full()) {
//         std::cout << id << ": wrench buffer not full: size: "
//                   << _wrench_buffers[id].size() << std::endl;
//         return false;
//       }
//     }
//   }
//   return true;
// }

// bool ManipServer::is_running() {
//   std::lock_guard<std::mutex> lock(_ctrl_mtx);
//   return _ctrl_flag_running;
// }

// const Eigen::MatrixXd ManipServer::get_camera_rgb(int k, int id) {
//   std::lock_guard<std::mutex> lock(_camera_rgb_buffer_mtxs[id]);
//   _camera_rgb_timestamps_ms[id] =
//       _camera_rgb_timestamp_ms_buffers[id].get_last_k(k);
//   return _camera_rgb_buffers[id].get_last_k(k);
// }

// const Eigen::MatrixXd ManipServer::get_wrench(int k, int id) {
//   std::lock_guard<std::mutex> lock(_wrench_buffer_mtxs[id]);
//   _wrench_timestamps_ms[id] = _wrench_timestamp_ms_buffers[id].get_last_k(k);
//   return _wrench_buffers[id].get_last_k(k);
// }

// const Eigen::MatrixXd ManipServer::get_pose(int k, int id) {
//   std::lock_guard<std::mutex> lock(_pose_buffer_mtxs[id]);
//   _pose_timestamps_ms[id] = _pose_timestamp_ms_buffers[id].get_last_k(k);
//   return _pose_buffers[id].get_last_k(k);
// }

// const Eigen::VectorXd ManipServer::get_camera_rgb_timestamps_ms(int id) {
//   return _camera_rgb_timestamps_ms[id];
// }
// const Eigen::VectorXd ManipServer::get_wrench_timestamps_ms(int id) {
//   return _wrench_timestamps_ms[id];
// }
// const Eigen::VectorXd ManipServer::get_pose_timestamps_ms(int id) {
//   return _pose_timestamps_ms[id];
// }

// double ManipServer::get_timestamp_now_ms() {
//   return _timer.toc_ms();
// }

// void ManipServer::set_high_level_maintain_position() {
//   if (!_config.run_robot_thread) {
//     return;
//   }
//   // clear existing targets
//   clear_cmd_buffer();

//   RUT::Vector7d pose_fb;
//   for (int id : _id_list) {
//     // get the current pose as the only new target
//     if (!_config.mock_hardware) {
//       robot_ptrs[id]->getCartesian(
//           pose_fb);  // use the current pose as the reference
//     }
//     set_target_pose(pose_fb, 200, id);
//     set_target_pose(pose_fb, 1000, id);
//   }
//   // wait for > 100ms before turn on high stiffness
//   // So that the internal target in the interpolation controller gets refreshed
//   std::this_thread::sleep_for(std::chrono::milliseconds(300));
//   for (int id : _id_list) {
//     std::lock_guard<std::mutex> lock(_controller_mtxs[id]);
//     // set the robot to have high stiffness, but still compliant
//     _controllers[id].setStiffnessMatrix(_stiffnesses_high[id]);
//     _controllers[id].setDampingMatrix(_dampings_high[id]);
//   }
// }

// void ManipServer::set_high_level_free_jogging() {
//   if (!_config.run_robot_thread) {
//     return;
//   }
//   for (int id : _id_list) {
//     std::lock_guard<std::mutex> lock(_controller_mtxs[id]);
//     // set the robot to be compliant
//     _controllers[id].setStiffnessMatrix(_stiffnesses_low[id]);
//     _controllers[id].setDampingMatrix(_dampings_low[id]);
//   }
// }

// void ManipServer::set_target_pose(const Eigen::Ref<RUT::Vector7d> pose,
//                                   double dt_in_future_ms, int robot_id) {
//   std::lock_guard<std::mutex> lock(_waypoints_buffer_mtxs[robot_id]);
//   _waypoints_buffers[robot_id].put(pose);
//   _waypoints_timestamp_ms_buffers[robot_id].put(
//       _timer.toc_ms() + dt_in_future_ms);  // 1s in the future
// }

// void ManipServer::set_force_controlled_axis(const RUT::Matrix6d& Tr, int n_af,
//                                             int robot_id) {
//   std::lock_guard<std::mutex> lock(_controller_mtxs[robot_id]);
//   _controllers[robot_id].setForceControlledAxis(Tr, n_af);
// }

// void ManipServer::set_stiffness_matrix(const RUT::Matrix6d& stiffness,
//                                        int robot_id) {
//   std::lock_guard<std::mutex> lock(_controller_mtxs[robot_id]);
//   _controllers[robot_id].setStiffnessMatrix(stiffness);
// }

// void ManipServer::clear_cmd_buffer() {
//   for (int id : _id_list) {
//     std::lock_guard<std::mutex> lock(_waypoints_buffer_mtxs[id]);
//     _waypoints_buffers[id].clear();
//     _waypoints_timestamp_ms_buffers[id].clear();
//   }
//   for (int id : _id_list) {
//     std::lock_guard<std::mutex> lock(_stiffness_buffer_mtxs[id]);
//     _stiffness_buffers[id].clear();
//     _stiffness_timestamp_ms_buffers[id].clear();
//   }
// }

// /*
//   1. Points in _waypoints_buffer are not yet scheduled to be executed. 
//   2. interpolation_controller will take the oldest N points away from _waypoints_buffer and _waypoints_timestamp_ms_buffer 
//     and interpolate them to generate a trajectory.
//   3. schedule_waypoints adds timed waypoints to the buffer following the procedures below:
//     a. remove input waypoints that are in the past.
//     b. remove existing waypoints that are newer than input waypoints.
//     c. Adds remaining of a to the end of b.
// */
// // #define DEBUG_WP_SCHEDULING
// void ManipServer::schedule_waypoints(const Eigen::MatrixXd& waypoints,
//                                      const Eigen::VectorXd& timepoints_ms,
//                                      int robot_id) {
//   double curr_time = _timer.toc_ms();
//   // check the shape of inputs
//   if (waypoints.rows() != 7) {
//     std::cerr << "[ManipServer][schedule_waypoints] Waypoints should have 7 "
//                  "rows. Exiting."
//               << std::endl;
//     return;
//   }
//   if (timepoints_ms.size() != waypoints.cols()) {
//     std::cerr << "[ManipServer][schedule_waypoints] Waypoints and "
//                  "timepoints_ms should have the same "
//                  "number of columns. Exiting."
//               << std::endl;
//     return;
//   }

// #ifdef DEBUG_WP_SCHEDULING
//   std::cout << "[ManipServer][schedule_waypoints] waypoints: \n"
//             << waypoints << std::endl;
//   std::cout << "[ManipServer][schedule_waypoints] timepoints_ms: \n"
//             << timepoints_ms.transpose() << std::endl;
//   std::cout << "[ManipServer][schedule_waypoints] curr_time: " << curr_time
//             << std::endl;
// #endif
//   /*
//    * a. Get rid of input waypoints that are in the past
//    */
//   int input_id_start = 0;
//   for (int i = 0; i < timepoints_ms.size(); i++) {
//     if (timepoints_ms(i) > curr_time) {
//       input_id_start = i;
//       break;
//     }
//   }
//   if (input_id_start >= timepoints_ms.size()) {
//     // all input points are in the past. Do nothing.
//     return;
//   }

//   {
//     std::lock_guard<std::mutex> lock(_waypoints_buffer_mtxs[robot_id]);
//     /*
//    * b. Get rid of existing waypoints that are newer than input waypoints
//    */
//     int existing_id_end = 0;
//     for (int i = 0; i < _waypoints_timestamp_ms_buffers[robot_id].size(); i++) {
//       if (_waypoints_timestamp_ms_buffers[robot_id][i] >
//           timepoints_ms(input_id_start)) {
//         existing_id_end = i;
//         break;
//       }
//     }
//     _waypoints_buffers[robot_id].remove_last_k(
//         _waypoints_buffers[robot_id].size() - existing_id_end);
//     _waypoints_timestamp_ms_buffers[robot_id].remove_last_k(
//         _waypoints_timestamp_ms_buffers[robot_id].size() - existing_id_end);
//     assert(_waypoints_buffers[robot_id].size() ==
//            _waypoints_timestamp_ms_buffers[robot_id].size());

//     /*
//    * c. Add remaining of a to the end of b
//    */
//     int input_id_end = timepoints_ms.size();
// #ifdef DEBUG_WP_SCHEDULING
//     std::cout << "[ManipServer][schedule_waypoints] input_id_start: "
//               << input_id_start << std::endl;
//     std::cout << "[ManipServer][schedule_waypoints] input_id_end: "
//               << input_id_end << std::endl;
// #endif
//     for (int i = input_id_start; i < input_id_end; i++) {
// #ifdef DEBUG_WP_SCHEDULING
//       std::cout << "[ManipServer][schedule_waypoints] Adding waypoint: "
//                 << waypoints.col(i).transpose()
//                 << " at time: " << timepoints_ms(i) << std::endl;
// #endif
//       _waypoints_buffers[robot_id].put(waypoints.col(i));
//       _waypoints_timestamp_ms_buffers[robot_id].put(timepoints_ms(i));
//     }
//   }
// }  // end function schedule_waypoints

// /*
//   1. Points in _waypoints_buffer are not yet scheduled to be executed. 
//   2. interpolation_controller will take the oldest N points away from _waypoints_buffer and _waypoints_timestamp_ms_buffer 
//     and interpolate them to generate a trajectory.
//   3. schedule_waypoints adds timed waypoints to the buffer following the procedures below:
//     a. remove input waypoints that are in the past.
//     b. remove existing waypoints that are newer than input waypoints.
//     c. Adds remaining of a to the end of b.
// */
// // #define DEBUG_STIFFNESS_SCHEDULING
// void ManipServer::schedule_stiffness(const Eigen::MatrixXd& stiffnesses,
//                                      const Eigen::VectorXd& timepoints_ms,
//                                      int robot_id) {
//   double curr_time = _timer.toc_ms();
//   // check the shape of inputs
//   if (stiffnesses.rows() != 6) {
//     std::cerr << "[ManipServer][schedule_stiffness] stiffnesses should have 6 "
//                  "rows. Exiting."
//               << std::endl;
//     return;
//   }
//   if (stiffnesses.cols() / timepoints_ms.size() != 6) {
//     std::cerr << "[ManipServer][schedule_stiffness] stiffnesses should have "
//                  "6x number of columns as timepoints_ms. Exiting."
//               << std::endl;
//     return;
//   }

// #ifdef DEBUG_STIFFNESS_SCHEDULING
//   std::cout << "[ManipServer][schedule_stiffness] stiffnesses: \n"
//             << stiffnesses << std::endl;
//   std::cout << "[ManipServer][schedule_stiffness] timepoints_ms: \n"
//             << timepoints_ms.transpose() << std::endl;
//   std::cout << "[ManipServer][schedule_stiffness] curr_time: " << curr_time
//             << std::endl;
// #endif
//   /*
//    * a. Get rid of inputs that are in the past
//    */
//   int input_id_start = 0;
//   for (int i = 0; i < timepoints_ms.size(); i++) {
//     if (timepoints_ms(i) > curr_time) {
//       input_id_start = i;
//       break;
//     }
//   }
//   if (input_id_start >= timepoints_ms.size()) {
//     // all input points are in the past. Do nothing.
//     return;
//   }

//   {
//     std::lock_guard<std::mutex> lock(_stiffness_buffer_mtxs[robot_id]);
//     /*
//      * b. Get rid of existing stiffness that are newer than input stiffness
//      */
//     int existing_id_end = 0;
//     for (int i = 0; i < _stiffness_timestamp_ms_buffers[robot_id].size(); i++) {
//       if (_stiffness_timestamp_ms_buffers[robot_id][i] >
//           timepoints_ms(input_id_start)) {
//         existing_id_end = i;
//         break;
//       }
//     }
//     _stiffness_buffers[robot_id].remove_last_k(
//         _stiffness_buffers[robot_id].size() - existing_id_end);
//     _stiffness_timestamp_ms_buffers[robot_id].remove_last_k(
//         _stiffness_timestamp_ms_buffers[robot_id].size() - existing_id_end);
//     assert(_stiffness_buffers[robot_id].size() ==
//            _stiffness_timestamp_ms_buffers[robot_id].size());
//     /*
//      * c. Add remaining of a to the end of b
//      */
//     int input_id_end = timepoints_ms.size();
// #ifdef DEBUG_STIFFNESS_SCHEDULING
//     std::cout << "[ManipServer][schedule_stiffness] input_id_start: "
//               << input_id_start << std::endl;
//     std::cout << "[ManipServer][schedule_stiffness] input_id_end: "
//               << input_id_end << std::endl;
// #endif
//     for (int i = input_id_start; i < input_id_end; i++) {
// #ifdef DEBUG_STIFFNESS_SCHEDULING
//       std::cout << "[ManipServer][schedule_stiffness] Adding stiffness:\n"
//                 << stiffnesses.middleCols<6>(6 * i)
//                 << " at time: " << timepoints_ms(i) << std::endl;
// #endif
//       _stiffness_buffers[robot_id].put(stiffnesses.middleCols<6>(i * 6));
//       _stiffness_timestamp_ms_buffers[robot_id].put(timepoints_ms(i));
//     }
//   }
// }  // end function schedule_stiffness

// void ManipServer::start_saving_data_for_a_new_episode() {
//   // create episode folders
//   std::vector<std::string> robot_json_file_names;
//   std::vector<std::string> wrench_json_file_names;
//   create_folder_for_new_episode(_config.data_folder, _id_list,
//                                 _ctrl_rgb_folders, robot_json_file_names,
//                                 wrench_json_file_names);

//   std::cout << "[main] New episode. rgb_folder_name: " << _ctrl_rgb_folders[0]
//             << std::endl;

//   // get rgb folder and low dim json file for saving data
//   for (int id : _id_list) {
//     _ctrl_robot_data_streams[id].open(robot_json_file_names[id]);
//     _ctrl_wrench_data_streams[id].open(wrench_json_file_names[id]);
//   }

//   {
//     std::lock_guard<std::mutex> lock(_ctrl_mtx);
//     _ctrl_flag_saving = true;
//   }
// }

// void ManipServer::stop_saving_data() {
//   std::lock_guard<std::mutex> lock(_ctrl_mtx);
//   _ctrl_flag_saving = false;
// }

// bool ManipServer::is_saving_data() {
//   bool is_saving = false;
//   for (int id : _id_list) {
//     is_saving = is_saving || _states_robot_thread_saving[id] ||
//                 _states_rgb_thread_saving[id] ||
//                 _states_wrench_thread_saving[id];
//   }
//   return is_saving;
// }

#include "table_top_manip/manip_server.h"
#include "helpers.hpp"

ManipServer::ManipServer(const std::string& config_path) {
  initialize(config_path);
}

ManipServer::~ManipServer() {}

bool ManipServer::initialize(const std::string& config_path) {
  std::cout << "[ManipServer] Initializing.\n";

  RUT::TimePoint time0 = _timer.tic();

  // read config files
  std::cout << "[ManipServer] Reading config files.\n";
  YAML::Node config;
  try {
    config = YAML::LoadFile(config_path);
    _config.deserialize(config);
  } catch (const std::exception& e) {
    std::cerr << "Failed to load the config file: " << e.what() << std::endl;
    return false;
  }

  if (_config.bimanual) {
    _id_list = {0, 1};
  } else {
    _id_list = {0};
  }

  std::cout << "_id_list: " << _id_list.size() << std::endl;

  // parameters to be obtained from config
  std::vector<int> wrench_publish_rate;

  std::cout << "[ManipServer] bimanual: " << _config.bimanual << std::endl;
  std::cout << "[ManipServer] Initialize each hardware interface.\n";

  // initialize hardwares
  if (!_config.mock_hardware) {
    for (int id : _id_list) {
      // Robot
      // TODO: support other robots
      URRTDE::URRTDEConfig robot_config;
      try {
        robot_config.deserialize(config["ur_rtde" + std::to_string(id)]);
      } catch (const std::exception& e) {
        std::cerr << "Failed to load the robot config file: " << e.what()
                  << std::endl;
        return false;
      }
      robot_ptrs.emplace_back(new URRTDE);
      URRTDE* urrtde_ptr = static_cast<URRTDE*>(robot_ptrs[id].get());
      if (!urrtde_ptr->init(time0, robot_config)) {
        std::cerr << "Failed to initialize UR RTDE for id " << id
                  << ". Exiting." << std::endl;
        return false;
      }

      // Camera
      if (_config.camera_selection == CameraSelection::GOPRO) {
        GoPro::GoProConfig gopro_config;
        try {
          gopro_config.deserialize(config["gopro" + std::to_string(id)]);
        } catch (const std::exception& e) {
          std::cerr << "Failed to load the GoPro config file: " << e.what()
                    << std::endl;
          return false;
        }
        camera_ptrs.emplace_back(new GoPro);
        GoPro* gopro_ptr = static_cast<GoPro*>(camera_ptrs[id].get());
        if (!gopro_ptr->init(time0, gopro_config)) {
          std::cerr << "Failed to initialize GoPro for id " << id
                    << ". Exiting." << std::endl;
          return false;
        }
      } else if (_config.camera_selection == CameraSelection::REALSENSE) {
        Realsense::RealsenseConfig realsense_config;
        try {
          realsense_config.deserialize(
              config["realsense" + std::to_string(id)]);
        } catch (const std::exception& e) {
          std::cerr << "Failed to load the Realsense config file: " << e.what()
                    << std::endl;
          return false;
        }
        camera_ptrs.emplace_back(new Realsense);
        Realsense* realsense_ptr =
            static_cast<Realsense*>(camera_ptrs[id].get());
        if (!realsense_ptr->init(time0, realsense_config)) {
          std::cerr << "Failed to initialize realsense for id " << id
                    << ". Exiting." << std::endl;
          return false;
        }
      } else {
        std::cerr << "Invalid camera selection. Exiting." << std::endl;
        return false;
      }

      // force sensor
      if (_config.force_sensing_mode == ForceSensingMode::FORCE_MODE_ATI) {
        ATINetft::ATINetftConfig ati_config;
        try {
          ati_config.deserialize(config["ati_netft" + std::to_string(id)]);
        } catch (const std::exception& e) {
          std::cerr << "Failed to load the ATI Netft config file: " << e.what()
                    << std::endl;
          return false;
        }
        force_sensor_ptrs.emplace_back(new ATINetft);
        ATINetft* ati_ptr = static_cast<ATINetft*>(force_sensor_ptrs[id].get());
        if (!ati_ptr->init(time0, ati_config)) {
          std::cerr << "Failed to initialize ATI Netft for id " << id
                    << ". Exiting." << std::endl;
          return false;
        }
        wrench_publish_rate.push_back(ati_config.publish_rate);
      } else if (_config.force_sensing_mode ==
                 ForceSensingMode::FORCE_MODE_ROBOTIQ) {
        RobotiqFTModbus::RobotiqFTModbusConfig robotiq_config;
        try {
          robotiq_config.deserialize(
              config["robotiq_ft_modbus" + std::to_string(id)]);
        } catch (const std::exception& e) {
          std::cerr << "Failed to load the Robotiq FT Modbus config file: "
                    << e.what() << std::endl;
          return false;
        }
        force_sensor_ptrs.emplace_back(new RobotiqFTModbus);
        RobotiqFTModbus* robotiq_ptr =
            static_cast<RobotiqFTModbus*>(force_sensor_ptrs[id].get());
        if (!robotiq_ptr->init(time0, robotiq_config)) {
          std::cerr << "Failed to initialize Robotiq FT Modbus for id " << id
                    << ". Exiting." << std::endl;
          return false;
        }
        wrench_publish_rate.push_back(robotiq_config.publish_rate);
      } else {
        std::cerr << "Invalid force sensing mode. Exiting." << std::endl;
        return false;
      }
    }
  } else {
    // mock hardware
    for (int id : _id_list) {
      wrench_publish_rate.push_back(7000);
    }
  }

  // initialize Admittance controller
  for (int id : _id_list) {
    AdmittanceController::AdmittanceControllerConfig admittance_config;
    try {
      deserialize(config["admittance_controller" + std::to_string(id)],
                  admittance_config);
    } catch (const std::exception& e) {
      std::cerr << "Failed to load the admittance controller config file: "
                << e.what() << std::endl;
      return false;
    }

    _controllers.emplace_back();
    _controller_mtxs.emplace_back();
    RUT::Vector7d pose = RUT::Vector7d::Zero();
    if (!_config.mock_hardware) {
      robot_ptrs[id]->getCartesian(pose);
    }
    if (!_controllers[id].init(time0, admittance_config, pose)) {
      std::cerr << "Failed to initialize admittance controller for id " << id
                << ". Exiting." << std::endl;
      return false;
    }
    RUT::Matrix6d Tr = RUT::Matrix6d::Identity();
    // The robot should not behave with any compliance during initialization.
    // The user needs to set the desired compliance afterwards.
    int n_af = 0;
    _controllers[id].setForceControlledAxis(Tr, n_af);

    _stiffnesses_high.push_back(admittance_config.compliance6d.stiffness);
    _stiffnesses_low.push_back(RUT::Matrix6d::Zero());
    _dampings_high.push_back(admittance_config.compliance6d.damping);
    _dampings_low.push_back(_config.low_damping);
  }

  // create the data buffers
  std::cout << "[ManipServer] Creating data buffers.\n";
  for (int id : _id_list) {
    _camera_rgb_buffers.push_back(RUT::DataBuffer<Eigen::MatrixXd>());
    _pose_buffers.push_back(RUT::DataBuffer<Eigen::VectorXd>());
    _wrench_buffers.push_back(RUT::DataBuffer<Eigen::VectorXd>());
    _waypoints_buffers.push_back(RUT::DataBuffer<Eigen::VectorXd>());
    _stiffness_buffers.push_back(RUT::DataBuffer<Eigen::MatrixXd>());

    _camera_rgb_timestamp_ms_buffers.push_back(RUT::DataBuffer<double>());
    _pose_timestamp_ms_buffers.push_back(RUT::DataBuffer<double>());
    _wrench_timestamp_ms_buffers.push_back(RUT::DataBuffer<double>());
    _waypoints_timestamp_ms_buffers.push_back(RUT::DataBuffer<double>());
    _stiffness_timestamp_ms_buffers.push_back(RUT::DataBuffer<double>());

    _camera_rgb_buffers[id].initialize(
        _config.rgb_buffer_size, 3 * _config.output_rgb_hw[0],
        _config.output_rgb_hw[1], "camera_rgb" + std::to_string(id));

    _pose_buffers[id].initialize(_config.pose_buffer_size, 7, 1,
                                 "pose" + std::to_string(id));
    _wrench_buffers[id].initialize(_config.wrench_buffer_size, 6, 1,
                                   "wrench" + std::to_string(id));

    _waypoints_buffers[id].initialize(-1, 7, 1,
                                      "waypoints" + std::to_string(id));
    _stiffness_buffers[id].initialize(-1, 6, 6,
                                      "stiffness" + std::to_string(id));

    _camera_rgb_timestamp_ms_buffers[id].initialize(
        _config.rgb_buffer_size, 1, 1,
        "camera_rgb" + std::to_string(id) + "_timestamp_ms");
    _pose_timestamp_ms_buffers[id].initialize(
        _config.pose_buffer_size, 1, 1,
        "pose" + std::to_string(id) + "_timestamp_ms");
    _wrench_timestamp_ms_buffers[id].initialize(
        _config.wrench_buffer_size, 1, 1,
        "wrench" + std::to_string(id) + "_timestamp_ms");
    _waypoints_timestamp_ms_buffers[id].initialize(
        -1, 1, 1, "waypoints" + std::to_string(id) + "_timestamp_ms");
    _stiffness_timestamp_ms_buffers[id].initialize(
        -1, 1, 1, "stiffness" + std::to_string(id) + "_timestamp_ms");
  }

  // initialize the buffer mutexes
  for (int id : _id_list) {
    _camera_rgb_buffer_mtxs.emplace_back();
    _pose_buffer_mtxs.emplace_back();
    _wrench_buffer_mtxs.emplace_back();
    _waypoints_buffer_mtxs.emplace_back();
    _stiffness_buffer_mtxs.emplace_back();
  }

  // initialize thread status variables
  for (int id : _id_list) {
    _states_robot_thread_ready.push_back(false);
    _states_rgb_thread_ready.push_back(false);
    _states_wrench_thread_ready.push_back(false);
    _states_robot_thread_saving.push_back(false);
    _states_rgb_thread_saving.push_back(false);
    _states_wrench_thread_saving.push_back(false);
    _states_robot_seq_id.push_back(0);
    _states_rgb_seq_id.push_back(0);
    _states_wrench_seq_id.push_back(0);
  }

  // initialize additional shared variables
  for (int id : _id_list) {
    _ctrl_rgb_folders.push_back("");
    _ctrl_robot_data_streams.push_back(std::ofstream());
    _ctrl_wrench_data_streams.push_back(std::ofstream());
    _color_mats.push_back(cv::Mat());
    _color_mat_mtxs.emplace_back();
    _poses_fb.push_back(Eigen::VectorXd());
    _poses_fb_mtxs.emplace_back();
    _camera_rgb_timestamps_ms.push_back(Eigen::VectorXd());
    _pose_timestamps_ms.push_back(Eigen::VectorXd());
    _wrench_timestamps_ms.push_back(Eigen::VectorXd());
  }

  // kickoff the threads
  _ctrl_flag_running = true;
  std::cout << "[ManipServer] Starting the threads.\n";
  for (int id : _id_list) {
    if (_config.run_rgb_thread) {
      _rgb_threads.emplace_back(&ManipServer::rgb_loop, this, std::ref(time0),
                                id);
    }
    if (_config.run_wrench_thread) {
      _wrench_threads.emplace_back(&ManipServer::wrench_loop, this,
                                   std::ref(time0), wrench_publish_rate[id],
                                   id);
    }
    if (_config.run_robot_thread) {
      _robot_threads.emplace_back(&ManipServer::robot_loop, this,
                                  std::ref(time0), id);
    }
  }
  if (_config.plot_rgb) {
    // pause 1s, then start the rgb plot thread
    std::this_thread::sleep_for(std::chrono::seconds(1));
    _rgb_plot_thread = std::thread(&ManipServer::rgb_plot_loop, this);
  }

  // wait for threads to be ready
  std::cout << "[ManipServer] Waiting for threads to be ready.\n";
  while (true) {
    bool all_ready = true;
    {
      std::lock_guard<std::mutex> lock(_ctrl_mtx);
      for (int id : _id_list) {
        if (_config.run_rgb_thread) {
          all_ready = all_ready && _states_rgb_thread_ready[id];
        }
        if (_config.run_wrench_thread) {
          all_ready = all_ready && _states_wrench_thread_ready[id];
        }
        if (_config.run_robot_thread) {
          all_ready = all_ready && _states_robot_thread_ready[id];
        }
      }
      if (_config.plot_rgb) {
        all_ready = all_ready && _state_plot_thread_ready;
      }
    }
    if (all_ready) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  std::cout << "[ManipServer] All threads are ready." << std::endl;
  std::cout << "[ManipServer] Done initialization." << std::endl;
  return true;
}

void ManipServer::join_threads() {
  std::cout << "[ManipServer]: Waiting for threads to join." << std::endl;
  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _ctrl_flag_running = false;
  }

  // join the threads
  if (_config.run_rgb_thread) {
    std::cout << "[ManipServer]: Waiting for rgb threads to join." << std::endl;
    for (auto& rgb_thread : _rgb_threads) {
      rgb_thread.join();
    }
  }
  if (_config.run_wrench_thread) {
    std::cout << "[ManipServer]: Waiting for wrench threads to join."
              << std::endl;
    for (auto& wrench_thread : _wrench_threads) {
      wrench_thread.join();
    }
  }
  if (_config.run_robot_thread) {
    std::cout << "[ManipServer]: Waiting for robot threads to join."
              << std::endl;
    for (auto& robot_thread : _robot_threads) {
      robot_thread.join();
    }
  }
  if (_config.plot_rgb) {
    std::cout << "[ManipServer]: Waiting for plotting thread to join."
              << std::endl;
    _rgb_plot_thread.join();
  }

  std::cout << "[ManipServer]: Threads have joined. Exiting." << std::endl;
}

bool ManipServer::is_ready() {
  for (int id : _id_list) {
    if (_config.run_rgb_thread) {
      std::lock_guard<std::mutex> lock(_camera_rgb_buffer_mtxs[id]);
      if (!_camera_rgb_buffers[id].is_full()) {
        std::cout << id << ": Camera RGB buffer not full: size: "
                  << _camera_rgb_buffers[id].size() << std::endl;
        return false;
      }
    }

    if (_config.run_robot_thread) {
      std::lock_guard<std::mutex> lock(_pose_buffer_mtxs[id]);
      if (!_pose_buffers[id].is_full()) {
        std::cout << id << ": Pose buffer not full: size: "
                  << _pose_buffers[id].size() << std::endl;
        return false;
      }
    }

    if (_config.run_wrench_thread) {
      std::lock_guard<std::mutex> lock(_wrench_buffer_mtxs[id]);
      if (!_wrench_buffers[id].is_full()) {
        std::cout << id << ": wrench buffer not full: size: "
                  << _wrench_buffers[id].size() << std::endl;
        return false;
      }
    }
  }
  return true;
}

bool ManipServer::is_running() {
  std::lock_guard<std::mutex> lock(_ctrl_mtx);
  return _ctrl_flag_running;
}

const Eigen::MatrixXd ManipServer::get_camera_rgb(int k, int id) {
  std::lock_guard<std::mutex> lock(_camera_rgb_buffer_mtxs[id]);
  _camera_rgb_timestamps_ms[id] =
      _camera_rgb_timestamp_ms_buffers[id].get_last_k(k);
  return _camera_rgb_buffers[id].get_last_k(k);
}

const Eigen::MatrixXd ManipServer::get_wrench(int k, int id) {
  std::lock_guard<std::mutex> lock(_wrench_buffer_mtxs[id]);
  _wrench_timestamps_ms[id] = _wrench_timestamp_ms_buffers[id].get_last_k(k);
  return _wrench_buffers[id].get_last_k(k);
}

const Eigen::MatrixXd ManipServer::get_pose(int k, int id) {
  std::lock_guard<std::mutex> lock(_pose_buffer_mtxs[id]);
  _pose_timestamps_ms[id] = _pose_timestamp_ms_buffers[id].get_last_k(k);
  return _pose_buffers[id].get_last_k(k);
}

const Eigen::VectorXd ManipServer::get_camera_rgb_timestamps_ms(int id) {
  return _camera_rgb_timestamps_ms[id];
}
const Eigen::VectorXd ManipServer::get_wrench_timestamps_ms(int id) {
  return _wrench_timestamps_ms[id];
}
const Eigen::VectorXd ManipServer::get_pose_timestamps_ms(int id) {
  return _pose_timestamps_ms[id];
}

double ManipServer::get_timestamp_now_ms() {
  return _timer.toc_ms();
}

void ManipServer::set_high_level_maintain_position() {
  if (!_config.run_robot_thread) {
    return;
  }
  // clear existing targets
  clear_cmd_buffer();

  RUT::Vector7d pose_fb;
  for (int id : _id_list) {
    // get the current pose as the only new target
    if (!_config.mock_hardware) {
      robot_ptrs[id]->getCartesian(
          pose_fb);  // use the current pose as the reference
    }
    set_target_pose(pose_fb, 200, id);
    set_target_pose(pose_fb, 1000, id);
  }
  // wait for > 100ms before turn on high stiffness
  // So that the internal target in the interpolation controller gets refreshed
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  for (int id : _id_list) {
    std::lock_guard<std::mutex> lock(_controller_mtxs[id]);
    // set the robot to have high stiffness, but still compliant
    _controllers[id].setStiffnessMatrix(_stiffnesses_high[id]);
    _controllers[id].setDampingMatrix(_dampings_high[id]);
  }
}

void ManipServer::set_high_level_free_jogging() {
  if (!_config.run_robot_thread) {
    return;
  }
  for (int id : _id_list) {
    std::lock_guard<std::mutex> lock(_controller_mtxs[id]);
    // set the robot to be compliant
    _controllers[id].setStiffnessMatrix(_stiffnesses_low[id]);
    _controllers[id].setDampingMatrix(_dampings_low[id]);
  }
}

void ManipServer::set_target_pose(const Eigen::Ref<RUT::Vector7d> pose,
                                  double dt_in_future_ms, int robot_id) {
  std::lock_guard<std::mutex> lock(_waypoints_buffer_mtxs[robot_id]);
  _waypoints_buffers[robot_id].put(pose);
  _waypoints_timestamp_ms_buffers[robot_id].put(
      _timer.toc_ms() + dt_in_future_ms);  // 1s in the future
}

void ManipServer::set_force_controlled_axis(const RUT::Matrix6d& Tr, int n_af,
                                            int robot_id) {
  std::lock_guard<std::mutex> lock(_controller_mtxs[robot_id]);
  _controllers[robot_id].setForceControlledAxis(Tr, n_af);
}

void ManipServer::set_stiffness_matrix(const RUT::Matrix6d& stiffness,
                                       int robot_id) {
  std::lock_guard<std::mutex> lock(_controller_mtxs[robot_id]);
  _controllers[robot_id].setStiffnessMatrix(stiffness);
}

void ManipServer::clear_cmd_buffer() {
  for (int id : _id_list) {
    std::lock_guard<std::mutex> lock(_waypoints_buffer_mtxs[id]);
    _waypoints_buffers[id].clear();
    _waypoints_timestamp_ms_buffers[id].clear();
  }
  for (int id : _id_list) {
    std::lock_guard<std::mutex> lock(_stiffness_buffer_mtxs[id]);
    _stiffness_buffers[id].clear();
    _stiffness_timestamp_ms_buffers[id].clear();
  }
}

/*
  1. Points in _waypoints_buffer are not yet scheduled to be executed. 
  2. interpolation_controller will take the oldest N points away from _waypoints_buffer and _waypoints_timestamp_ms_buffer 
    and interpolate them to generate a trajectory.
  3. schedule_waypoints adds timed waypoints to the buffer following the procedures below:
    a. remove input waypoints that are in the past.
    b. remove existing waypoints that are newer than input waypoints.
    c. Adds remaining of a to the end of b.
*/
// #define DEBUG_WP_SCHEDULING
void ManipServer::schedule_waypoints(const Eigen::MatrixXd& waypoints,
                                     const Eigen::VectorXd& timepoints_ms,
                                     int robot_id) {
  double curr_time = _timer.toc_ms();
  // check the shape of inputs
  if (waypoints.rows() != 7) {
    std::cerr << "[ManipServer][schedule_waypoints] Waypoints should have 7 "
                 "rows. Exiting."
              << std::endl;
    return;
  }
  if (timepoints_ms.size() != waypoints.cols()) {
    std::cerr << "[ManipServer][schedule_waypoints] Waypoints and "
                 "timepoints_ms should have the same "
                 "number of columns. Exiting."
              << std::endl;
    return;
  }

#ifdef DEBUG_WP_SCHEDULING
  std::cout << "[ManipServer][schedule_waypoints] waypoints: \n"
            << waypoints << std::endl;
  std::cout << "[ManipServer][schedule_waypoints] timepoints_ms: \n"
            << timepoints_ms.transpose() << std::endl;
  std::cout << "[ManipServer][schedule_waypoints] curr_time: " << curr_time
            << std::endl;
#endif
  /*
   * a. Get rid of input waypoints that are in the past
   */
  int input_id_start = 0;
  for (int i = 0; i < timepoints_ms.size(); i++) {
    if (timepoints_ms(i) > curr_time) {
      input_id_start = i;
      break;
    }
  }
  if (input_id_start >= timepoints_ms.size()) {
    // all input points are in the past. Do nothing.
    return;
  }

  {
    std::lock_guard<std::mutex> lock(_waypoints_buffer_mtxs[robot_id]);
    /*
   * b. Get rid of existing waypoints that are newer than input waypoints
   */
    int existing_id_end = 0;
    for (int i = 0; i < _waypoints_timestamp_ms_buffers[robot_id].size(); i++) {
      if (_waypoints_timestamp_ms_buffers[robot_id][i] >
          timepoints_ms(input_id_start)) {
        existing_id_end = i;
        break;
      }
    }
    _waypoints_buffers[robot_id].remove_last_k(
        _waypoints_buffers[robot_id].size() - existing_id_end);
    _waypoints_timestamp_ms_buffers[robot_id].remove_last_k(
        _waypoints_timestamp_ms_buffers[robot_id].size() - existing_id_end);
    assert(_waypoints_buffers[robot_id].size() ==
           _waypoints_timestamp_ms_buffers[robot_id].size());

    /*
   * c. Add remaining of a to the end of b
   */
    int input_id_end = timepoints_ms.size();
#ifdef DEBUG_WP_SCHEDULING
    std::cout << "[ManipServer][schedule_waypoints] input_id_start: "
              << input_id_start << std::endl;
    std::cout << "[ManipServer][schedule_waypoints] input_id_end: "
              << input_id_end << std::endl;
#endif
    for (int i = input_id_start; i < input_id_end; i++) {
#ifdef DEBUG_WP_SCHEDULING
      std::cout << "[ManipServer][schedule_waypoints] Adding waypoint: "
                << waypoints.col(i).transpose()
                << " at time: " << timepoints_ms(i) << std::endl;
#endif
      _waypoints_buffers[robot_id].put(waypoints.col(i));
      _waypoints_timestamp_ms_buffers[robot_id].put(timepoints_ms(i));
    }
  }
}  // end function schedule_waypoints

/*
  1. Points in _waypoints_buffer are not yet scheduled to be executed. 
  2. interpolation_controller will take the oldest N points away from _waypoints_buffer and _waypoints_timestamp_ms_buffer 
    and interpolate them to generate a trajectory.
  3. schedule_waypoints adds timed waypoints to the buffer following the procedures below:
    a. remove input waypoints that are in the past.
    b. remove existing waypoints that are newer than input waypoints.
    c. Adds remaining of a to the end of b.
*/
// #define DEBUG_STIFFNESS_SCHEDULING
void ManipServer::schedule_stiffness(const Eigen::MatrixXd& stiffnesses,
                                     const Eigen::VectorXd& timepoints_ms,
                                     int robot_id) {
  double curr_time = _timer.toc_ms();
  // check the shape of inputs
  if (stiffnesses.rows() != 6) {
    std::cerr << "[ManipServer][schedule_stiffness] stiffnesses should have 6 "
                 "rows. Exiting."
              << std::endl;
    return;
  }
  if (stiffnesses.cols() / timepoints_ms.size() != 6) {
    std::cerr << "[ManipServer][schedule_stiffness] stiffnesses should have "
                 "6x number of columns as timepoints_ms. Exiting."
              << std::endl;
    return;
  }

#ifdef DEBUG_STIFFNESS_SCHEDULING
  std::cout << "[ManipServer][schedule_stiffness] stiffnesses: \n"
            << stiffnesses << std::endl;
  std::cout << "[ManipServer][schedule_stiffness] timepoints_ms: \n"
            << timepoints_ms.transpose() << std::endl;
  std::cout << "[ManipServer][schedule_stiffness] curr_time: " << curr_time
            << std::endl;
#endif
  /*
   * a. Get rid of inputs that are in the past
   */
  int input_id_start = 0;
  for (int i = 0; i < timepoints_ms.size(); i++) {
    if (timepoints_ms(i) > curr_time) {
      input_id_start = i;
      break;
    }
  }
  if (input_id_start >= timepoints_ms.size()) {
    // all input points are in the past. Do nothing.
    return;
  }

  {
    std::lock_guard<std::mutex> lock(_stiffness_buffer_mtxs[robot_id]);
    /*
     * b. Get rid of existing stiffness that are newer than input stiffness
     */
    int existing_id_end = 0;
    for (int i = 0; i < _stiffness_timestamp_ms_buffers[robot_id].size(); i++) {
      if (_stiffness_timestamp_ms_buffers[robot_id][i] >
          timepoints_ms(input_id_start)) {
        existing_id_end = i;
        break;
      }
    }
    _stiffness_buffers[robot_id].remove_last_k(
        _stiffness_buffers[robot_id].size() - existing_id_end);
    _stiffness_timestamp_ms_buffers[robot_id].remove_last_k(
        _stiffness_timestamp_ms_buffers[robot_id].size() - existing_id_end);
    assert(_stiffness_buffers[robot_id].size() ==
           _stiffness_timestamp_ms_buffers[robot_id].size());
    /*
     * c. Add remaining of a to the end of b
     */
    int input_id_end = timepoints_ms.size();
#ifdef DEBUG_STIFFNESS_SCHEDULING
    std::cout << "[ManipServer][schedule_stiffness] input_id_start: "
              << input_id_start << std::endl;
    std::cout << "[ManipServer][schedule_stiffness] input_id_end: "
              << input_id_end << std::endl;
#endif
    for (int i = input_id_start; i < input_id_end; i++) {
#ifdef DEBUG_STIFFNESS_SCHEDULING
      std::cout << "[ManipServer][schedule_stiffness] Adding stiffness:\n"
                << stiffnesses.middleCols<6>(6 * i)
                << " at time: " << timepoints_ms(i) << std::endl;
#endif
      _stiffness_buffers[robot_id].put(stiffnesses.middleCols<6>(i * 6));
      _stiffness_timestamp_ms_buffers[robot_id].put(timepoints_ms(i));
    }
  }
}  // end function schedule_stiffness

void ManipServer::start_saving_data_for_a_new_episode() {
  // create episode folders
  std::vector<std::string> robot_json_file_names;
  std::vector<std::string> wrench_json_file_names;
  create_folder_for_new_episode(_config.data_folder, _id_list,
                                _ctrl_rgb_folders, robot_json_file_names,
                                wrench_json_file_names);

  std::cout << "[main] New episode. rgb_folder_name: " << _ctrl_rgb_folders[0]
            << std::endl;

  // get rgb folder and low dim json file for saving data
  for (int id : _id_list) {
    _ctrl_robot_data_streams[id].open(robot_json_file_names[id]);
    _ctrl_wrench_data_streams[id].open(wrench_json_file_names[id]);
  }

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _ctrl_flag_saving = true;
  }
}

void ManipServer::stop_saving_data() {
  std::lock_guard<std::mutex> lock(_ctrl_mtx);
  _ctrl_flag_saving = false;
}

bool ManipServer::is_saving_data() {
  bool is_saving = false;
  for (int id : _id_list) {
    is_saving = is_saving || _states_robot_thread_saving[id] ||
                _states_rgb_thread_saving[id] ||
                _states_wrench_thread_saving[id];
  }
  return is_saving;
}