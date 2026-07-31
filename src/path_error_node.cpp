// src/path_error_node.cpp
//
// State estimator - encoder odometry
// Integrates (v, omega) → global pose (x, y, theta), then projects on
// the reference path to produce (e_l, e_theta, kappa).
//
// Reference path = stadium (obround): two lines of length 2*path_a
// connected by two semicircles of radius path_R, centered at (path_xc, path_yc),
// with the major axis along the X-axis. Traversed in the clockwise direction.
//
// e_l convention: positive toward the LEFT of the path.
//
// RATE: integration is triggered by the arrival of /joint_states
// (~985 Hz, published from the Gazebo physics loop)

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

using std::placeholders::_1;

class PathErrorNode : public rclcpp::Node
{
public:
  PathErrorNode() : Node("path_error_node")
  {
    // Physical Parameters
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("wheel_separation", 0.102);
    get_parameter("wheel_radius", r_);
    get_parameter("wheel_separation", L_);

    // Reference path : stadium
    declare_parameter("path_xc", 0.0);   // stadium center
    declare_parameter("path_yc", 0.0);   // stadium center
    declare_parameter("path_R",  0.5);   // semicircles radius
    declare_parameter("path_a",  5.0);   // half the length of the right side
    get_parameter("path_xc", xc_);
    get_parameter("path_yc", yc_);
    get_parameter("path_R",  R_path_);
    get_parameter("path_a",  a_);

    // Initial position of the estimator
    declare_parameter("x0", 0.0);
    declare_parameter("y0", -0.5);
    declare_parameter("theta0", 0.0);
    get_parameter("x0", x_);
    get_parameter("y0", y_);
    get_parameter("theta0", theta_);

    // Decimation of /pose_est only
    declare_parameter("pose_decimation", 5);
    get_parameter("pose_decimation", pose_decim_max_);

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&PathErrorNode::joint_state_cb, this, _1));

    path_error_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/path_error", 10);

    pose_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/pose_est", 10);

    // Safety measures in case a robot spawns on the path or tangent
    double e_l0 = 0.0, e_theta0 = 0.0, kappa0 = 0.0;
    compute_error(e_l0, e_theta0, kappa0);

    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        rcl_interfaces::msg::SetParametersResult res;
        res.successful = true;
        for (const auto & p : params) {
          const auto & n = p.get_name();
          if (n == "path_a") {
            if (p.as_double() <= 0.0) { res.successful = false; res.reason = "path_a should be > 0"; }
            else a_ = p.as_double();
          }
          else if (n == "path_R") {
            if (p.as_double() <= 0.0) { res.successful = false; res.reason = "path_R should be > 0"; }
            else R_path_ = p.as_double();
          }
          else if (n == "path_xc") xc_ = p.as_double();
          else if (n == "path_yc") yc_ = p.as_double();
        }
        return res;
      });
  }

private:

  // Get angular velocity, calculate dt, and perform integration
  void joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == "left_wheel_joint")  omega_l_ = msg->velocity[i];
      if (msg->name[i] == "right_wheel_joint") omega_r_ = msg->velocity[i];
    }

    /// dt measured based on the message timestamp
    const rclcpp::Time stamp(msg->header.stamp);
    if (stamp.nanoseconds() == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "/joint_states stamp missing");
      return;
    }
    if (!t_prev_init_) { t_prev_ = stamp; t_prev_init_ = true; return; }

    const double dt = (stamp - t_prev_).seconds();
    t_prev_ = stamp;
    if (dt <= 0.0) return;

    integrate_and_publish(dt);
  }

  // Get distance, angle between robot and path, and curve
  // Returns false if the pose is on the "backbone"
  bool compute_error(double & e_l, double & e_theta, double & kappa) const
  {
    //FORM TO FOLLOW
    //xc_ and yc_ : stadium center
    //a_ : "backbone" without semicircles
    //R_path_ : semicircles radius
    //th_p : angle de la tangente au chemin pris

    //ROBOT
    //x_ et y_ : Calculated robot position - odometry
    //theta_ : robot orientation - odometry
    //px, py = "the “backbone” point closest to the robot
    //dx, dy — the vector pointing from the local center toward the robot
    //d — the length of this vector = distance between the robot and the local center

    //Output
    //e_l : distance between robot and "backbone"
    //e_theta : diff th_p (curve) et theta (robot)
    //kappa : radius of curvature, 0 ou 1/R_path

    // Projection onto the central segment (backbone)
    const double px = xc_ + std::clamp(x_ - xc_, -a_, a_);
    const double py = yc_;

    const double dx = x_ - px;
    const double dy = y_ - py;
    const double d  = std::hypot(dx, dy);

    // Safety against NaN
    if (d < 1e-6) return false;

    // Distance robot <-> path
    e_l = R_path_ - d;

    // Tangent to the path
    const double th_p = std::atan2(dy, dx) + M_PI_2;

    // Robot diff angle <-> path + module to stay between -pi and pi
    e_theta = std::atan2(std::sin(theta_ - th_p), std::cos(theta_ - th_p));

    // Straights : kappa = 0. Semicircles : kappa = 1/R.
    kappa = (std::fabs(x_ - xc_) > a_) ? 1.0 / R_path_ : 0.0;

    return true;
  }

  // Convert wheel speeds to position, then convert position to trajectory error
  void integrate_and_publish(double dt)
  {
    // Direct Kinematics - Robot Linear and Angular Velocity
    const double v     = r_ * (omega_l_ + omega_r_) / 2.0;
    const double omega = r_ * (omega_r_ - omega_l_) / L_;

    // Odometry
    // x_ et y_ : calculated robot position
    // theta_ : calculated robot orientation
    x_     += v * std::cos(theta_) * dt;
    y_     += v * std::sin(theta_) * dt;
    theta_ += omega * dt;

    // Normalize theta to [-pi, pi]
    theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

    if (++pose_decim_ >= pose_decim_max_) {
      pose_decim_ = 0;
      std_msgs::msg::Float64MultiArray pose;
      pose.data = {x_, y_, theta_};
      pose_pub_->publish(pose);
    }

    // Get distance, angle error, and curvature
    double e_l = 0.0, e_theta = 0.0, kappa = 0.0;
    if (!compute_error(e_l, e_theta, kappa)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Placed on the stadium's backbone, indefinite projection.");
      return;
    }

    std_msgs::msg::Float64MultiArray out;
    out.data = {e_l, e_theta, kappa};
    path_error_pub_->publish(out);
  }

  // Parameters
  double r_ = 0.033, L_ = 0.102;

  // Reference path (stage)
  double xc_ = 0.0, yc_ = 0.0, R_path_ = 0.5, a_ = 0.5;

  double omega_l_ = 0.0, omega_r_ = 0.0;
  double x_ = 0.0, y_ = 0.0, theta_ = 0.0;

  int pose_decim_ = 0, pose_decim_max_ = 5;
  bool t_prev_init_ = false;
  rclcpp::Time t_prev_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr path_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pose_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathErrorNode>());
  rclcpp::shutdown();
  return 0;
}