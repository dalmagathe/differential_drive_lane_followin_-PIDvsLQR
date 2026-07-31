#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using std::placeholders::_1;

// Generic PID. Derivative calculated based on the measured value (not
// on the error) to avoid a derivative kick when the setpoint changes
// Anti-windup via back-calculation based on output saturation
class PID
{
public:
  PID(double kp, double ki, double kd, double dt,
      double out_min = -std::numeric_limits<double>::infinity(),
      double out_max =  std::numeric_limits<double>::infinity())
  : kp_(kp), ki_(ki), kd_(kd), dt_(dt), out_min_(out_min), out_max_(out_max){}

  void reset()
  {
    integral_ = 0.0;
    prev_measurement_.reset();
  }

  void set_gains(double kp, double ki, double kd) { kp_ = kp; ki_ = ki; kd_ = kd; }
  void set_output_limits(double out_min, double out_max) { out_min_ = out_min; out_max_ = out_max; }

  double update(double measurement,
                std::optional<double> setpoint = std::nullopt,
                std::optional<double> dt = std::nullopt)
  {
    if (setpoint.has_value()) setpoint_ = setpoint.value();
    const double dt_used = dt.value_or(dt_);

    const double error = setpoint_ - measurement;
    const double p_term = kp_ * error;

    integral_ += error * dt_used;
    const double i_term = ki_ * integral_;

    double raw_derivative = 0.0;
    if (prev_measurement_.has_value()) {
      raw_derivative = -(measurement - prev_measurement_.value()) / dt_used;
    }
    prev_measurement_ = measurement;
    const double d_term = kd_ * raw_derivative;

    const double output  = p_term + i_term + d_term;
    const double clamped = clamp(output);

    if (clamped != output && ki_ != 0.0) {
      integral_ -= (output - clamped) / ki_;  // anti-windup
    }
    return clamped;
  }

private:
  double clamp(double value) const { return std::min(out_max_, std::max(out_min_, value)); }

  double kp_, ki_, kd_, dt_;
  double out_min_, out_max_;
  double integral_ {0.0};
  double setpoint_ {0.0};
  std::optional<double> prev_measurement_;
};

// 3 lopps
//
//     pid_pos_ (external, slow)  : e_l         -> desired heading e_theta_ref
//     pid_cap_ (internal, fast)  : e_theta_ref -> V_rot
//
//   COMMON MODE
//     pid_spd_ : (omega_L + omega_R)/2 -> V_avg
//
// Output : V_L = V_nom_L + V_avg - V_rot
//          V_R = V_nom_R + V_avg + V_rot

class PidControllerNode : public rclcpp::Node
{
public:
  PidControllerNode(): Node("pid_controller_node")
  {
    // Physical Parameters
    declare_parameter("Ke", 0.478);
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("wheel_base", 0.102);
    declare_parameter("vr", 0.2);
    declare_parameter("v_max", 5.0);

    // Hz
    declare_parameter("decimation", 5);

    // PID gains
    declare_parameter("kp_pos", 3.0);
    declare_parameter("ki_pos", 0.0);
    declare_parameter("kd_pos", 0.0);
    declare_parameter("kp_cap", 3.0);
    declare_parameter("ki_cap", 0.0);
    declare_parameter("kd_cap", 0.0);
    declare_parameter("kp_spd", 1.0);
    declare_parameter("ki_spd", 0.0);

    // Loop Saturation
    declare_parameter("theta_ref_max", 1.0);  // rad
    declare_parameter("v_rot_max", 1.5);      // V
    declare_parameter("v_avg_max", 1.0);      // V

    double theta_ref_max, kp_pos, ki_pos, kd_pos, kp_cap, ki_cap, kd_cap;
    double kp_spd, ki_spd, v_rot_max, v_avg_max;

    get_parameter("decimation", decim_max_);
    get_parameter("Ke", Ke_);
    get_parameter("wheel_radius", r_);
    get_parameter("wheel_base", L_);
    get_parameter("vr", vr_);
    get_parameter("v_max", u_max_);
    get_parameter("theta_ref_max", theta_ref_max);
    get_parameter("kp_pos", kp_pos);
    get_parameter("ki_pos", ki_pos);
    get_parameter("kd_pos", kd_pos);
    get_parameter("kp_cap", kp_cap);
    get_parameter("ki_cap", ki_cap);
    get_parameter("kd_cap", kd_cap);
    get_parameter("kp_spd", kp_spd);
    get_parameter("ki_spd", ki_spd);
    get_parameter("v_rot_max", v_rot_max);
    get_parameter("v_avg_max", v_avg_max);

    // Construction dt = rollback value only: dt is measured at each
    // cycle and explicitly passed to update()
    const double dt_nominal = 0.005;

    pid_pos_ = std::make_unique<PID>(kp_pos, ki_pos, kd_pos, dt_nominal,
                                     -theta_ref_max, theta_ref_max);
    pid_cap_ = std::make_unique<PID>(kp_cap, ki_cap, kd_cap, dt_nominal,
                                     -v_rot_max, v_rot_max);
    pid_spd_ = std::make_unique<PID>(kp_spd, ki_spd, 0.0, dt_nominal,
                                     -v_avg_max, v_avg_max);

    // Subscription
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&PidControllerNode::joint_state_cb, this, _1));

    path_error_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/path_error", 10,
      std::bind(&PidControllerNode::path_error_cb, this, _1));

    // Publisher
    voltage_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/motor_voltage_cmd", 10);
  }

private:
  // Loop entry point: triggered by the measurement, not by a timer
  void joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == "left_wheel_joint")  omega_l_ = msg->velocity[i];
      if (msg->name[i] == "right_wheel_joint") omega_r_ = msg->velocity[i];
    }

    if (++decim_ < decim_max_) return;
    decim_ = 0;

    /// dt measured based on the message timestamp
    const rclcpp::Time stamp(msg->header.stamp);
    if (stamp.nanoseconds() == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "/joint_states stamp missing");
      return;
    }

    // 1st cycle
    if (!t_prev_init_) { t_prev_ = stamp; t_prev_init_ = true; return; }

    const double dt = (stamp - t_prev_).seconds();
    t_prev_ = stamp;
    if (dt <= 0.0) return;

    control_loop(dt);
  }

  // Get path error
  void path_error_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() == 3) {
      e_l_     = msg->data[0];
      e_theta_ = msg->data[1];
      kappa_   = msg->data[2];
      path_error_received_ = true;
    }
  }

  void control_loop(double dt)
  {
    // /joint_states received?
    if (!path_error_received_) return;

    // Nominal, b = 0 => i = 0 => V_nom = Ke * omega_nom
    const double omega_L_nom = vr_ * (1.0 - kappa_ * L_ / 2.0) / r_;
    const double omega_R_nom = vr_ * (1.0 + kappa_ * L_ / 2.0) / r_;
    const double V_nom_L = Ke_ * omega_L_nom;
    const double V_nom_R = Ke_ * omega_R_nom;

    // Cascaded: position -> heading
    const double e_theta_ref = pid_pos_->update(e_l_, 0.0, dt);
    const double v_rot       = pid_cap_->update(e_theta_, e_theta_ref, dt);

    // Average speed
    const double omega_moy     = 0.5 * (omega_l_ + omega_r_);
    const double omega_moy_ref = 0.5 * (omega_L_nom + omega_R_nom);
    const double v_avg         = pid_spd_->update(omega_moy, omega_moy_ref, dt);

    // Speed reassembly + actuator saturation
    const double v_l = std::clamp(V_nom_L + v_avg - v_rot, -u_max_, u_max_);
    const double v_r = std::clamp(V_nom_R + v_avg + v_rot, -u_max_, u_max_);

    std_msgs::msg::Float64MultiArray msg;
    msg.data = {v_l, v_r};
    voltage_pub_->publish(msg);
  }

  std::unique_ptr<PID> pid_pos_, pid_cap_, pid_spd_;

  double Ke_ {0.478}, r_ {0.033}, L_ {0.102}, vr_ {0.2}, u_max_ {5.0};

  double omega_l_ {0.0}, omega_r_ {0.0};
  double e_l_ {0.0}, e_theta_ {0.0}, kappa_ {0.0};
  bool path_error_received_ {false};

  int decim_ {0};
  int decim_max_ {5};
  bool t_prev_init_ {false};
  rclcpp::Time t_prev_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr path_error_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr voltage_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PidControllerNode>());
  rclcpp::shutdown();
  return 0;
}