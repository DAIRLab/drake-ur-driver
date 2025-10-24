#include <pthread.h>

#include <algorithm>

#include <gflags/gflags.h>
#include <lcm/lcm-cpp.hpp>
#include <ur_client_library/ur/ur_driver.h>

#include "drake/common/drake_throw.h"
#include "drake/common/find_runfiles.h"
#include "drake/common/text_logging.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/parsing/process_model_directives.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake_ur_driver/lcmt_ur_command.hpp"
#include "drake_ur_driver/lcmt_ur_status.hpp"

using namespace urcl;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::parsing::LoadModelDirectives;

DEFINE_string(robot_ip_address, "192.168.56.101",
              "Address of the shop floor interface");
DEFINE_string(lcm_url, "", "LCM URL for UR driver");
DEFINE_string(lcm_command_channel, "UR_COMMAND",
              "Channel to listen for lcmt_ur_command messages on");
DEFINE_string(lcm_status_channel, "UR_STATUS",
              "Channel to publish lcmt_ur_status messages on");
DEFINE_string(control_mode, "position",
              "Choose from: status_only, velocity, position (default), "
              "tcp_pose, tcp_velocity");
DEFINE_bool(
    use_mbp, false,
    "Use Drake MbP for dynamics computation for adding limit checks. If true, "
    "use urdf_path to provide the model. If false, skip limit checks.");
DEFINE_string(urdf_path, "drake-ur-driver/resources/ur10.urdf",
              "Model file to use.");
DEFINE_int32(scale_velocity_limits, -1,
             "Velocity joint limit factor k (k * <joint_velocity_limit> / "
             "100.0). range [1-100]");
DEFINE_int32(scale_joint_limits, -1,
             "Joint limit factor k (k * <joint_limit> / 100.0). range [1-100]");
DEFINE_int32(command_stop_limit, 5,
             "Maximum number of messages before stopping commands.");
DEFINE_double(
    expire_sec, 0.1,
    "How much delay is allowed for messages to be allowed. Converted to "
    "usec, must be non-negative + finite.");

constexpr int DOF = 6;
inline const std::string SCRIPT_FILE =
    "drake-ur-driver/resources/external_control.urscript";
inline const std::string OUTPUT_RECIPE_FILE =
    "drake-ur-driver/resources/rtde_output_recipe.txt";
inline const std::string INPUT_RECIPE_FILE =
    "drake-ur-driver/resources/rtde_input_recipe.txt";

template <typename T, std::size_t N>
void CopyArrayToVector(std::vector<T>* dest, const std::array<T, N>& src) {
  dest->resize(N);
  memcpy(dest->data(), src.data(), N * sizeof(T));
}

enum class ControlMode {
  /** Only publish status */
  kStatusOnly,
  /** Command velocities via Joint impedance */
  kVelocity,
  /** Command positions and velocities via Joint impedance */
  kPosition,
  kTCPPose,
  kTCPVelocity,
};

ControlMode ToControlMode(std::string value) {
  if (value == "status_only") {
    return ControlMode::kStatusOnly;
  } else if (value == "velocity") {
    return ControlMode::kVelocity;
  } else if (value == "position") {
    return ControlMode::kPosition;
  } else if (value == "tcp_pose") {
    return ControlMode::kTCPPose;
  } else if (value == "tcp_velocity") {
    return ControlMode::kTCPVelocity;
  } else {
    throw std::runtime_error("Invalid ControlMode: " + value);
  }
}

class URDriverRunner {
 public:
  URDriverRunner(std::string robot_ip_address, ControlMode control_mode,
                 const std::string& lcm_url,
                 const std::string& lcm_command_channel,
                 const std::string& lcm_status_channel,
                 std::unique_ptr<MultibodyPlant<double>> plant,
                 uint32_t expire_usec,
                 const std::string& output_recipe_file = OUTPUT_RECIPE_FILE,
                 const std::string& input_recipe_file = INPUT_RECIPE_FILE,
                 const std::string& script_file = SCRIPT_FILE)
      : control_mode_(control_mode),
        lcm_(lcm_url),
        lcm_command_channel_(lcm_command_channel),
        lcm_status_channel_(lcm_status_channel),
        plant_(std::move(plant)),
        expire_usec_(expire_usec) {
    UrDriverConfiguration driver_config;
    driver_config.robot_ip = robot_ip_address;
    driver_config.script_file = script_file;
    driver_config.output_recipe_file = output_recipe_file;
    driver_config.input_recipe_file = input_recipe_file;
    driver_config.headless_mode = true;
    driver_config.handle_program_state = [](bool state) {
      std::cout << "Robot program state changed: "
                << (state ? "running" : "not running") << std::endl;
    };
    ur_driver_ = std::make_unique<UrDriver>(driver_config);
  }

  ~URDriverRunner() { stop(); }

  int run() {
    if (!lcm_.good()) return 1;

    lcm::Subscription* sub = lcm_.subscribe(
        lcm_command_channel_, &URDriverRunner::getCommandMessage, this);
    sub->setQueueCapacity(100);
    stop_status_thread_ = false;
    status_thread_ = std::thread(&URDriverRunner::handleStatusMessage, this);
    stop_command_thread_ = false;
    command_thread_ = std::thread(&URDriverRunner::handleCommandMessage, this);
    // Figure out a way to set this thread to real-time priority
    // pthread_setschedparam(status_thread_.native_handle(), SCHED_FIFO,
    //                        nullptr);
    {
      std::unique_lock<std::mutex> lock(handlers_mutex_);
      handlers_cv_.wait(lock);
      stop();
    }
    return 0;
  }

  int stop() {
    ur_driver_->stopControl();
    stop_command_thread_ = true;
    if (command_thread_.joinable()) {
      command_thread_.join();
    }
    stop_status_thread_ = true;
    if (status_thread_.joinable()) {
      status_thread_.join();
    }
    return 0;
  }

 private:
  void getCommandMessage(const lcm::ReceiveBuffer*, const std::string&,
                         const drake_ur_driver::lcmt_ur_command* command) {
    if (command->control_mode_expected != static_cast<int8_t>(control_mode_)) {
      drake::log()->warn(
          "Received command with unexpected control mode [skipping command]."
          " Expected {}, got {} ",
          static_cast<int8_t>(control_mode_), command->control_mode_expected);
      return;
    }

    const int64_t now = std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();
    if (std::abs(now - command->utime) > expire_usec_) {
      drake::log()->warn("packet too old [{} usec], skipping",
                         now - command->utime);
      return;
    }
    command_ = *command;
  }

  void handleCommandMessage() {
    while (!stop_command_thread_) {
      // Check if LCM message waiting 
      if (lcm_.handleTimeout(1000 / 125) <= 0 || !command_.has_value()) {
        // If we haven't received any valid messages yet, keep waiting
        if (!first_message_received_) continue;

        no_message_count_++;

        // If we haven't received a message in N consecutive cycles, stop
        // sending commands
        if (no_message_count_ >= FLAGS_command_stop_limit) {
          // Stop sending commands after the limit is reached
          if (no_message_count_ == FLAGS_command_stop_limit) {
            if (prev_command_) prev_command_.reset();
            drake::log()->warn(
                "No command message received in the last {} cycles. Stopping "
                "commands to robot.",
                FLAGS_command_stop_limit);
          }
          ur_driver_->writeKeepalive(RobotReceiveTimeout::millisec(100));
          continue;
        }

        // Else, repeat the previous command
        command_ = prev_command_;
      } 
      else {
        // We received a message, reset the no message counter
        if (!first_message_received_) first_message_received_ = true;
        no_message_count_ = 0;
      }

      // Clear out any other pending messages
      while (lcm_.handleTimeout(0) > 0) {
      }

      // Handle the command
      switch (control_mode_) {
        case ControlMode::kStatusOnly:
          // do nothing
          break;
        case ControlMode::kVelocity: {
          ur_driver_->writeJointCommand(std::to_array(command_->joint_velocity),
                                        comm::ControlMode::MODE_SPEEDJ,
                                        RobotReceiveTimeout::millisec(100));
          break;
        }
        case ControlMode::kPosition: {
          ur_driver_->writeJointCommand(std::to_array(command_->joint_position),
                                        comm::ControlMode::MODE_SERVOJ,
                                        RobotReceiveTimeout::millisec(100));
          break;
        }
        case ControlMode::kTCPPose: {
          ur_driver_->writeJointCommand(std::to_array(command_->tcp_pose),
                                        comm::ControlMode::MODE_POSE,
                                        RobotReceiveTimeout::millisec(100));
          break;
        }
        case ControlMode::kTCPVelocity: {
          ur_driver_->writeJointCommand(std::to_array(command_->tcp_velocity),
                                        comm::ControlMode::MODE_SPEEDL,
                                        RobotReceiveTimeout::millisec(100));
          break;
        }
        default:
          drake::log()->error(
              "Invalid ControlMode detected, Stopping command thread.");
          stop_command_thread_ = true;
          break;
      }
      prev_command_ = command_;
      command_.reset();
    }
    handlers_cv_.notify_one();
  }

  void handleStatusMessage() {
    // Start RTDE communication
    ur_driver_->startRTDECommunication();
    while (!stop_status_thread_) {
      // Wait for next package @ 125Hz
      std::unique_ptr<rtde_interface::DataPackage> data_pkg =
          ur_driver_->getDataPackage();
      if (!data_pkg) {
        drake::log()->warn("Error getting data package from robot");
        continue;
      }

      // Parse data package
      drake_ur_driver::lcmt_ur_status status_msg;
      status_msg.utime =
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::system_clock::now().time_since_epoch())
              .count();
      double timestamp = 0.0;
      if (!data_pkg->getData("timestamp", timestamp)) {
        drake::log()->warn("Could not find timestamp in data package");
      }

      status_msg.robot_utime = timestamp * 1e6;  // convert to microseconds
      std::unordered_map<std::string, double*> data_map;
      data_map["actual_q"] = status_msg.joint_position;
      data_map["target_q"] = status_msg.joint_position_desired;
      data_map["actual_qd"] = status_msg.joint_velocity;
      data_map["target_qd"] = status_msg.joint_velocity_desired;
      data_map["target_qdd"] = status_msg.joint_acceleration_desired;
      data_map["actual_current"] = status_msg.joint_current;
      data_map["target_current"] = status_msg.joint_current_desired;
      data_map["actual_TCP_pose"] = status_msg.tcp_pose;
      data_map["target_TCP_pose"] = status_msg.tcp_pose_desired;
      data_map["actual_TCP_speed"] = status_msg.tcp_speed;
      data_map["target_TCP_speed"] = status_msg.tcp_speed_desired;
      data_map["actual_TCP_force"] = status_msg.tcp_force;
      // assuming 6dof robot
      for (auto [key, value] : data_map) {
        vector6d_t arr;
        if (!data_pkg->getData(key, arr)) {
          drake::log()->warn("Could not find {} in data package", key);
        }
        memcpy(value, arr.data(), sizeof(double) * DOF);
      }

      if (!data_pkg->getData("robot_mode", status_msg.robot_mode)) {
        drake::log()->warn("Could not find robot_mode in data package");
      }

      // Publish status message
      lcm_.publish(lcm_status_channel_, &status_msg);

      // Check for limits violations
      if (IsLimitsExceeded(status_msg)) {
        drake::log()->error(
            "Joint/Workspace limits exceeded, Exiting status thread.");
        stop_status_thread_ = true;
        break;
      }
    }
    // Notify the command thread to stop as well
    handlers_cv_.notify_one();
  }

  bool IsLimitsExceeded(drake_ur_driver::lcmt_ur_status& status) {
    // If no plant is provided, skip the checks
    if (!plant_) return false;

    bool limits_exceeded = false;

    if (FLAGS_scale_velocity_limits > 0) {
      // check velocity limits
      double velocity_scale_factor =
          (std::min(std::max(FLAGS_scale_velocity_limits, 1), 100) / 100.0);
      drake::VectorX<double> upper_velocity_limit =
          plant_->GetVelocityUpperLimits() * velocity_scale_factor;
      drake::VectorX<double> lower_velocity_limit =
          plant_->GetVelocityLowerLimits() * velocity_scale_factor;

      for (int i = 0; i < DOF; ++i) {
        if (status.joint_velocity[i] > upper_velocity_limit[i] ||
            status.joint_velocity[i] < lower_velocity_limit[i]) {
          drake::log()->error(
              "Joint {} velocity {} exceeds limit of [{}, {}] (factor {})", i,
              status.joint_velocity[i], upper_velocity_limit[i],
              lower_velocity_limit[i], FLAGS_scale_velocity_limits);
          limits_exceeded = true;
        }
      }
    }

    if (FLAGS_scale_joint_limits > 0) {
      // check position limits
      double joint_scale_factor =
          (std::min(std::max(FLAGS_scale_joint_limits, 1), 100) / 100.0);
      drake::VectorX<double> upper_joint_limit =
          plant_->GetPositionUpperLimits() * joint_scale_factor;
      drake::VectorX<double> lower_joint_limit =
          plant_->GetPositionLowerLimits() * joint_scale_factor;

      for (int i = 0; i < DOF; ++i) {
        if (status.joint_position[i] > upper_joint_limit[i] ||
            status.joint_position[i] < lower_joint_limit[i]) {
          drake::log()->error(
              "Joint {} position {} exceeds limit of [{}, {}] (factor {})", i,
              status.joint_position[i], upper_joint_limit[i],
              lower_joint_limit[i], FLAGS_scale_joint_limits);
          limits_exceeded = true;
        }
      }
    }
    return limits_exceeded;
  }

  ControlMode control_mode_{ControlMode::kStatusOnly};

  lcm::LCM lcm_;
  const std::string lcm_command_channel_;
  const std::string lcm_status_channel_;
  std::optional<drake_ur_driver::lcmt_ur_command> command_;
  std::optional<drake_ur_driver::lcmt_ur_command> prev_command_;

  std::unique_ptr<UrDriver> ur_driver_;

  std::thread status_thread_;
  std::atomic<bool> stop_status_thread_{false};
  std::thread command_thread_;
  std::atomic<bool> stop_command_thread_{false};
  std::mutex handlers_mutex_;
  std::condition_variable handlers_cv_;

  int no_message_count_{0};
  bool first_message_received_{false};

  std::unique_ptr<MultibodyPlant<double>> plant_;
  const uint32_t expire_usec_{};
};

// N.B. Using a resource path allows us to locate
// any Bazel resource, be it Drake or another repository.
std::string GetPathOrThrow(const drake::RlocationOrError& result) {
  if (!result.error.empty()) {
    throw std::runtime_error(result.error);
  }
  DRAKE_DEMAND(!result.abspath.empty());
  return result.abspath;
}

std::unique_ptr<MultibodyPlant<double>> MaybeLoadPlant() {
  if (!FLAGS_use_mbp) {
    return nullptr;
  }
  const std::string model_file = FLAGS_urdf_path;
  //   GetPathOrThrow(drake::FindRunfile(FLAGS_urdf_path));
  const double time_step = 0.0;
  auto plant = std::make_unique<MultibodyPlant<double>>(time_step);
  drake::multibody::Parser parser(plant.get());
  parser.AddModels(model_file);
  plant->Finalize();
  return plant;
}

int DoMain() {
  DRAKE_THROW_UNLESS(FLAGS_robot_ip_address != "");
  DRAKE_THROW_UNLESS(FLAGS_lcm_command_channel != "");
  DRAKE_THROW_UNLESS(FLAGS_lcm_status_channel != "");
  DRAKE_THROW_UNLESS(FLAGS_expire_sec >= 0.0 &&
                     std::isfinite(FLAGS_expire_sec));

  const uint32_t expire_usec = static_cast<uint32_t>(FLAGS_expire_sec * 1e6);

  const ControlMode mode = ToControlMode(FLAGS_control_mode);

  URDriverRunner runner(FLAGS_robot_ip_address, mode, FLAGS_lcm_url,
                        FLAGS_lcm_command_channel, FLAGS_lcm_status_channel,
                        MaybeLoadPlant(), expire_usec);
  runner.run();
  return 0;
}

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return DoMain();
}