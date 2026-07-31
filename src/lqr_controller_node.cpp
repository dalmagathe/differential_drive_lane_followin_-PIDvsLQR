// src/lqr_controller_node.cpp
//
// LQR controller - path tracking
// x = [e_l, e_theta, Δω_L, Δω_R], u = [V_L, V_R]
//
// u_LQR = -K·x is a deviation from the nominal voltage of each wheel.
// The actual voltage supplied ist V_nom_L/R + u_LQR.
//
// kappa is a variable (path stage: 0 on straight sections, 1/R on curves) and
// is received at each cycle via /path_error[2]. The setpoint is therefore recalculated in
// control_loop(), not fixed at startup.
//
// K remains unchanged: the linearized model does not contain kappa.
//
// RATE: The loop is triggered by the arrival of /joint_states (~985 Hz,
// published from the Gazebo physics loop), decimated by a factor of 5 -> ~197 Hz

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

using std::placeholders::_1;

struct RobotParameters {
  double wheel_radius = 0.033;  // m
  double wheel_base   = 0.102;  // m
  double vr           = 0.2;    // m/s    - reference speed
  double U_max        = 5.0;    // V      - actuator saturation
  double Ke           = 0.478;  // V·s/rad
  double kappa        = 0.0;    // m^-1    - standard curvature, received from the estimator

  // Nominal operating point, per wheel (calculated)
  double omega_L_nom = 0.0, omega_R_nom = 0.0;
  double V_nom_L = 0.0, V_nom_R = 0.0;

  // b = 0 => i = 0 => V_nom = Ke * omega_nom
  void compute_nominal() {
    omega_L_nom = vr * (1.0 - kappa * wheel_base / 2.0) / wheel_radius;
    omega_R_nom = vr * (1.0 + kappa * wheel_base / 2.0) / wheel_radius;
    V_nom_L = Ke * omega_L_nom;
    V_nom_R = Ke * omega_R_nom;
  }
};

class LqrControllerNode : public rclcpp::Node
{
public:
  LqrControllerNode() : Node("lqr_controller_node")
  {
    // Physical Parameters
    declare_parameter("Ke", 0.478);
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("wheel_base", 0.102);
    declare_parameter("vr", 0.2);
    declare_parameter("v_max", 5.0);

    // Hz
    declare_parameter("decimation", 5);

    // Gains LQR : matrice K [e_l, e_theta, omega_L, omega_R] -> [v_l, v_r]
    declare_parameter("K", std::vector<double>(8, 0.0));

    get_parameter("Ke", rbt_prm_.Ke);
    get_parameter("wheel_radius", rbt_prm_.wheel_radius);
    get_parameter("wheel_base", rbt_prm_.wheel_base);
    get_parameter("vr", rbt_prm_.vr);
    get_parameter("v_max", rbt_prm_.U_max);
    get_parameter("decimation", decim_max_);

    // LQR K gain
    std::vector<double> K_vector;
    get_parameter("K", K_vector);
    if (K_vector.size() != 8) {
      RCLCPP_ERROR(get_logger(),
                   "Check K size (2x4), received %zu",
                   K_vector.size());
    } else {
      for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 4; ++j)
          K_[i][j] = K_vector[i * 4 + j];
    }

    // Subscription
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&LqrControllerNode::joint_state_cb, this, _1));

    path_error_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/path_error", 10,
      std::bind(&LqrControllerNode::path_error_cb, this, _1));

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

    control_loop();
  }

  // Get path error
  void path_error_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() == 3) {
      e_l_     = msg->data[0];
      e_theta_ = msg->data[1];
      rbt_prm_.kappa = msg->data[2];
      path_error_received_ = true;
    }
  }

  void control_loop()
  {
    // /joint_states received?
    if (!path_error_received_) {return;}

    // Nominal operating point, recalculated using the current kappa
    rbt_prm_.compute_nominal();

    // Nominal operating point, recalculated using the current kappa
    const std::array<double, 4> x = {
        e_l_, e_theta_,
        omega_l_ - rbt_prm_.omega_L_nom,
        omega_r_ - rbt_prm_.omega_R_nom};

    // u = -K·x (deviation from V_nom)
    double u_l = 0.0, u_r = 0.0;
    for (int j = 0; j < 4; ++j) {
      u_l -= K_[0][j] * x[j];
      u_r -= K_[1][j] * x[j];
    }

    // Full command
    double v_l = rbt_prm_.V_nom_L + u_l;
    double v_r = rbt_prm_.V_nom_R + u_r;

    // Check saturation
    v_l = std::clamp(v_l, -rbt_prm_.U_max, rbt_prm_.U_max);
    v_r = std::clamp(v_r, -rbt_prm_.U_max, rbt_prm_.U_max);

    std_msgs::msg::Float64MultiArray out;
    out.data = {v_l, v_r};
    voltage_pub_->publish(out);
  }

  // Robot param and LQR gain
  RobotParameters rbt_prm_;
  double K_[2][4] = {{0}};

  // State act
  double e_l_ = 0.0, e_theta_ = 0.0, omega_l_ = 0.0, omega_r_ = 0.0;

  int decim_ = 0, decim_max_ = 5;
  bool path_error_received_ = false;
  bool first_cycle_ = true;
  double kappa_prev_ = 0.0;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr path_error_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr voltage_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LqrControllerNode>());
  rclcpp::shutdown();
  return 0;
}