// hil_torque_mixer_node.cpp
//
// Mixes τ_pid (from velocity_pid_node) and τ_hydro (from chrono_flap_node in HIL mode)
// with independent watchdogs and a hard safety clamp, then publishes the combined command
// to the real motor effort controller.
//
// Safety properties:
//   - Independent watchdog timers on both input topics.
//   - If either input goes stale, its contribution is zeroed (fail-safe to τ_pid only or 0).
//   - Hard clip on τ_total before publishing.
//   - On node shutdown, publishes a final [0.0] command.
//   - enable_load runtime toggle (also via ~/enable_load SetBool service).
//   - Default failure mode: zero output.
//
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"

class HilTorqueMixerNode : public rclcpp::Node
{
public:
  HilTorqueMixerNode()
  : Node("hil_torque_mixer_node"),
    tau_pid_(0.0),
    tau_hydro_(0.0)
  {
    // ── Parameters ────────────────────────────────────────────────────────────────────────────
    this->declare_parameter<std::string>("pid_torque_topic",
      "/velocity_pid_node/torque_command");
    this->declare_parameter<std::string>("load_torque_topic",
      "/chrono_flap_node/load_torque");
    this->declare_parameter<std::string>("output_topic",
      "/motor_effort_controller/commands");
    this->declare_parameter<double>("output_rate_hz", 200.0);
    this->declare_parameter<double>("hard_clip_nm",   0.5);
    this->declare_parameter<double>("pid_timeout_s",  0.2);
    this->declare_parameter<double>("load_timeout_s", 0.2);
    this->declare_parameter<bool>  ("enable_load",    true);

    // Read
    pid_torque_topic_  = this->get_parameter("pid_torque_topic").as_string();
    load_torque_topic_ = this->get_parameter("load_torque_topic").as_string();
    output_topic_      = this->get_parameter("output_topic").as_string();
    output_rate_hz_    = this->get_parameter("output_rate_hz").as_double();
    hard_clip_nm_      = this->get_parameter("hard_clip_nm").as_double();
    pid_timeout_s_     = this->get_parameter("pid_timeout_s").as_double();
    load_timeout_s_    = this->get_parameter("load_timeout_s").as_double();
    enable_load_       = this->get_parameter("enable_load").as_bool();

    // Validate
    if (hard_clip_nm_ <= 0.0)  { hard_clip_nm_  = 0.5;  RCLCPP_WARN(this->get_logger(), "hard_clip_nm reset to 0.5"); }
    if (pid_timeout_s_ <= 0.0) { pid_timeout_s_ = 0.2;  RCLCPP_WARN(this->get_logger(), "pid_timeout_s reset to 0.2"); }
    if (load_timeout_s_ <= 0.0){ load_timeout_s_= 0.2;  RCLCPP_WARN(this->get_logger(), "load_timeout_s reset to 0.2"); }
    if (output_rate_hz_ <= 0.0){ output_rate_hz_= 200.0; RCLCPP_WARN(this->get_logger(), "output_rate_hz reset to 200"); }

    // ── Parameter callback for runtime tunable params ─────────────────────────────────────────
    on_set_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        return on_validate_parameters(params);
      });
    post_set_handle_ = this->add_post_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        for (const auto & p : params) {
          if (p.get_name() == "hard_clip_nm")   { hard_clip_nm_   = p.as_double(); }
          if (p.get_name() == "pid_timeout_s")  { pid_timeout_s_  = p.as_double(); }
          if (p.get_name() == "load_timeout_s") { load_timeout_s_ = p.as_double(); }
          if (p.get_name() == "enable_load")    { enable_load_    = p.as_bool(); }
        }
      });

    // ── Subscriptions ─────────────────────────────────────────────────────────────────────────
    pid_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      pid_torque_topic_,
      rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
        if (!msg->data.empty()) {
          tau_pid_         = msg->data[0];
          pid_last_stamp_  = this->now();
          pid_received_    = true;
        }
      });

    load_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      load_torque_topic_,
      rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
        tau_hydro_         = msg->data;
        load_last_stamp_   = this->now();
        load_received_     = true;
      });

    // ── Publisher ─────────────────────────────────────────────────────────────────────────────
    output_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(output_topic_, 10);

    // ── Enable service ────────────────────────────────────────────────────────────────────────
    enable_service_ = this->create_service<std_srvs::srv::SetBool>(
      "~/enable_load",
      [this](
        const std_srvs::srv::SetBool::Request::SharedPtr req,
        std_srvs::srv::SetBool::Response::SharedPtr res) {
        enable_load_ = req->data;
        res->success = true;
        res->message = enable_load_ ? "Load torque ENABLED" : "Load torque DISABLED";
        RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
      });

    // ── Timer ─────────────────────────────────────────────────────────────────────────────────
    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / output_rate_hz_));
    timer_ = this->create_wall_timer(period_ns, [this]() { tick(); });

    RCLCPP_INFO(this->get_logger(),
      "HilTorqueMixerNode started: pid='%s' + load='%s' → '%s' @ %.0f Hz, clip=%.3f Nm",
      pid_torque_topic_.c_str(), load_torque_topic_.c_str(),
      output_topic_.c_str(), output_rate_hz_, hard_clip_nm_);
  }

  ~HilTorqueMixerNode()
  {
    // On shutdown: publish a safe zero command
    if (output_pub_) {
      std_msgs::msg::Float64MultiArray zero;
      zero.data.push_back(0.0);
      output_pub_->publish(zero);
    }
  }

private:
  void tick()
  {
    const rclcpp::Time now = this->now();

    // Watchdog: PID input
    double effective_pid = tau_pid_;
    if (!pid_received_) {
      effective_pid = 0.0;
    } else {
      const double age = (now - pid_last_stamp_).seconds();
      if (age > pid_timeout_s_) {
        effective_pid = 0.0;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "HilMixer: PID input stale (%.3f s > %.3f s) — zeroing τ_pid.", age, pid_timeout_s_);
      }
    }

    // Watchdog: load input
    double effective_load = 0.0;
    if (enable_load_) {
      if (!load_received_) {
        effective_load = 0.0;
      } else {
        const double age = (now - load_last_stamp_).seconds();
        if (age > load_timeout_s_) {
          effective_load = 0.0;
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "HilMixer: load input stale (%.3f s > %.3f s) — zeroing τ_hydro.", age, load_timeout_s_);
        } else {
          effective_load = tau_hydro_;
        }
      }
    }

    const double tau_total = std::clamp(
      effective_pid + effective_load, -hard_clip_nm_, hard_clip_nm_);

    std_msgs::msg::Float64MultiArray out;
    out.data.push_back(tau_total);
    output_pub_->publish(out);
  }

  rcl_interfaces::msg::SetParametersResult on_validate_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    static const std::set<std::string> kImmutableParams = {
      "pid_torque_topic", "load_torque_topic", "output_topic", "output_rate_hz"};

    for (const auto & param : parameters) {
      if (kImmutableParams.count(param.get_name())) {
        result.successful = false;
        result.reason = "Parameter '" + param.get_name() + "' cannot be changed at runtime.";
        return result;
      }
      if (param.get_name() == "hard_clip_nm" || param.get_name() == "pid_timeout_s"
          || param.get_name() == "load_timeout_s") {
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE
            || param.as_double() <= 0.0) {
          result.successful = false;
          result.reason = param.get_name() + " must be positive.";
          return result;
        }
      }
    }
    return result;
  }

  // ── Parameters ────────────────────────────────────────────────────────────────────────────────
  std::string pid_torque_topic_{"/velocity_pid_node/torque_command"};
  std::string load_torque_topic_{"/chrono_flap_node/load_torque"};
  std::string output_topic_{"/motor_effort_controller/commands"};
  double      output_rate_hz_{200.0};
  double      hard_clip_nm_{0.5};
  double      pid_timeout_s_{0.2};
  double      load_timeout_s_{0.2};
  bool        enable_load_{true};

  // ── Runtime state ─────────────────────────────────────────────────────────────────────────────
  double       tau_pid_{0.0};
  double       tau_hydro_{0.0};
  rclcpp::Time pid_last_stamp_;
  rclcpp::Time load_last_stamp_;
  bool         pid_received_{false};
  bool         load_received_{false};

  // ── ROS interfaces ────────────────────────────────────────────────────────────────────────────
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pid_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr           load_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    output_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr                enable_service_;
  rclcpp::TimerBase::SharedPtr                                      timer_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr   on_set_handle_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_handle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HilTorqueMixerNode>());
  rclcpp::shutdown();
  return 0;
}
