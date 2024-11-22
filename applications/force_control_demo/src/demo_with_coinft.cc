#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>
#include <coinft/coin_ft.h>
#include <force_control/admittance_controller.h>
#include <force_control/config_deserialize.h>
#include <unistd.h>
#include <ur_rtde/ur_rtde.h>
#include <yaml-cpp/yaml.h>

// Added for plotting
#include <ftxui/screen/screen.hpp>
#include <deque>
#include <thread>

class ForcePlotter {
private:
    static const size_t WINDOW_SIZE = 50;
    static const size_t PLOT_HEIGHT = 10;
    std::deque<std::array<double, 6>> data;
    std::array<double, 6> min_vals{}, max_vals{};
    ftxui::Screen screen;
    
    void updateMinMax(const std::array<double, 6>& values) {
        for (size_t i = 0; i < 6; i++) {
            min_vals[i] = std::min(min_vals[i], values[i]);
            max_vals[i] = std::max(max_vals[i], values[i]);
        }
    }

public:
    ForcePlotter() : screen(ftxui::Screen::Create(
        /*width=*/120,
        /*height=*/40
    )) {
        for (auto& val : min_vals) val = std::numeric_limits<double>::max();
        for (auto& val : max_vals) val = std::numeric_limits<double>::lowest();
    }

    void update(const std::vector<double>& ftData) {
        std::array<double, 6> current_data;
        std::copy_n(ftData.begin(), 6, current_data.begin());
        
        data.push_back(current_data);
        updateMinMax(current_data);
        
        if (data.size() > WINDOW_SIZE) {
            data.pop_front();
        }

        // Create graphs
        using namespace ftxui;
        std::vector<Element> graphs;
        const std::array<std::string, 6> labels = {"Fx", "Fy", "Fz", "Tx", "Ty", "Tz"};
        
        for (size_t i = 0; i < 6; i++) {
            std::vector<int> plot_data;
            for (const auto& d : data) {
                double normalized = (d[i] - min_vals[i]) / (max_vals[i] - min_vals[i]);
                plot_data.push_back(static_cast<int>(normalized * PLOT_HEIGHT));
            }
            
            auto graph = vbox({
                text(labels[i]),
                graph(std::move(plot_data)) | flex,
                separator(),
            });
            graphs.push_back(graph);
        }

        auto document = hbox({
            vbox({graphs[0], graphs[1], graphs[2]}) | flex,
            separator(),
            vbox({graphs[3], graphs[4], graphs[5]}) | flex,
        });

        screen = Screen::Create(screen.dimx(), screen.dimy());
        Render(screen, document);
        screen.Print();
    }
};

// Added for plotting

Eigen::MatrixXd deserialize_matrix(const YAML::Node& node) {
  int nr = node.size();
  int nc = node[0].size();
  Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(nr, nc);
  for (int r = 0; r < nr; ++r) {
    for (int c = 0; c < nc; ++c) {
      mat(r, c) = node[r][c].as<double>();
    }
  }
  return mat;
}

// load eigen
template <typename T>
T deserialize_vector(const YAML::Node& node) {
  std::vector<double> q = node.as<std::vector<double>>();
  return Eigen::Map<T, Eigen::Unaligned>(q.data(), q.size());
}

int main() {
  URRTDE::URRTDEConfig robot_config;
  AdmittanceController::AdmittanceControllerConfig admittance_config;

  // open file
  const std::string CONFIG_PATH =
      "/Users/chuerpan/Documents/repo/umiFT/submodules/hardware_interfaces/applications/force_control_demo/"
      "config/force_control_demo.yaml";
  YAML::Node config{};
  // CoinFT configs
  // TODO: replace with CoinFT config serialization
  bool ft_use_coinft = false;
  std::string coinft_port;
  unsigned int coinft_baud_rate;
  std::string coinft_calibration_file;
  std::vector<double> coinft_PoseSensorTool;
  RUT::Matrix6d adj_sensor_tool = RUT::Matrix6d::Identity();
  try {
    config = YAML::LoadFile(CONFIG_PATH);
    robot_config.deserialize(config["ur_rtde"]);
    deserialize(config["admittance_controller"], admittance_config);
    ft_use_coinft = config["ft_use_coinft"].as<bool>();

    coinft_port = config["coin_ft"]["port"].as<std::string>();
    coinft_baud_rate = config["coin_ft"]["baud_rate"].as<unsigned int>();
    coinft_calibration_file =
        config["coin_ft"]["calibration_file"].as<std::string>();
    coinft_PoseSensorTool =
        config["coin_ft"]["PoseSensorTool"].as<std::vector<double>>();
    RUT::Vector7d pose_sensor_tool;
    pose_sensor_tool = RUT::Vector7d::Map(coinft_PoseSensorTool.data(), 7);
    adj_sensor_tool = RUT::SE32Adj(RUT::pose2SE3(pose_sensor_tool));
  } catch (const std::exception& e) {
    std::cerr << "Failed to load the config file: " << e.what() << std::endl;
    return -1;
  }

  URRTDE robot;
  AdmittanceController controller;
  RUT::Timer timer;
  RUT::TimePoint time0 = timer.tic();
  RUT::Vector7d pose, pose_ref, pose_cmd;
  RUT::Vector6d wrench_S, wrench_T, wrench_WTr;
  wrench_WTr.setZero();

  robot.init(time0, robot_config);
  robot.getCartesian(pose);

  CoinFT sensor(coinft_port, coinft_baud_rate, coinft_calibration_file);
  sensor.startStreaming();

  std::cout << "Starting in 2 seconds ..." << std::endl;
  sleep(2.0);

  controller.init(time0, admittance_config, pose);

  RUT::Matrix6d Tr = RUT::Matrix6d::Identity();
  // RUT::Matrix6d Tr;
  // // clang-format off
  // Tr <<
  //       0, 1, 0, 0, 0, 0,
  //       0, 0, 0, 1, 0, 0,
  //       0, 0, 0, 0, 1, 0,
  //       0, 0, 0, 0, 0, 1,
  //       1, 0, 0, 0, 0, 0,
  //       0, 0, 1, 0, 0, 0;
  // // clang-format on
  int n_af = 6;
  controller.setForceControlledAxis(Tr, n_af);

  pose_ref = pose;
  wrench_T.setZero();

  timer.tic();

  ForcePlotter plotter;

  while (true) {
    RUT::TimePoint t_start = robot.rtde_init_period();
    // Update robot status
    robot.getCartesian(pose);

    // read wrench
    // robot.getWrenchTool(wrench);
    std::vector<double> ftData = sensor.getLatestData();

    if (!ftData.empty()) {
      plotter.update(ftData);
      std::cout << "[" << timer.toc_ms() << " ms] "
                << "Force/Torque Data: ";
      for (const auto& value : ftData) {
        std::cout << value << " ";
      }
      std::cout << std::endl;
    } else {
      std::cout << "No data available yet." << std::endl;
      continue;
    }
    for (size_t i = 0; i < 6; i++) {
      wrench_S[i] = ftData[i];
    }
    // Transform sensor wrench to robot base frame
    wrench_T = adj_sensor_tool.transpose() * wrench_S;

    controller.setRobotStatus(pose, wrench_T);

    // Update robot reference
    controller.setRobotReference(pose_ref, wrench_WTr);

    // Compute the control output
    controller.step(pose_cmd);

    if (!robot.streamCartesian(pose_cmd)) {
      printf("streamCartesian failed\n");
      break;
    }

    double dt = timer.toc_ms();
    printf("t = %f, wrench: %f %f %f %f %f %f\n", dt, wrench_T[0], wrench_T[1],
           wrench_T[2], wrench_T[3], wrench_T[4], wrench_T[5]);

    if (dt > 300000)
      break;

    robot.rtde_wait_period(t_start);
  }

  return 0;
}