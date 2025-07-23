#include <experimental/string_view>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/detail/laser_scan__struct.hpp>
#include <string>

class FollowWall : public rclcpp::Node {
private:
  //* Constants on program
  static constexpr const char *SUB_NAME = "scan";
  static constexpr const char *PUB_NAME = "cmd_vel";
  static constexpr int GLOBAL_QOS = 10;
  static constexpr int RIGHT_WALL_ANGLE = 270;
  static constexpr int FRONT_WALL_ANGLE = 0;
  static constexpr float MIN_DIST_RIGHT_WALL = 0.22;
  static constexpr float MIN_DIST_FRONT_WALL = 0.5;
  static constexpr float LINEAR_VEL = 0.07;
  static constexpr float ANGULAR_VEL = 0.23;
  static constexpr float DIVING_VEL = 0.6;
  static constexpr float NO_ANGULAR_VEL = 0;

  //* Shortening names
  using LaserScan = sensor_msgs::msg::LaserScan;
  using TwistRobot = geometry_msgs::msg::Twist;

  enum State { IDLE, GET_CLOSER, GET_FARTHER, DIVE_LEFT };
  State curr_state_{IDLE};

  //* Subscriber and publishers
  rclcpp::Subscription<LaserScan>::SharedPtr sub_scan_;
  rclcpp::Publisher<TwistRobot>::SharedPtr pub_vel_;

  /**
   * @brief Get the state of the robot based on ranges
   *
   * @param ranges is an array of size 360, where each angle is the distance to
   * an object at that angle
   * @return state of the robot
   */
  State get_state(const std::vector<float> &ranges) {
    float right_wall_distance = ranges[RIGHT_WALL_ANGLE];
    float front_robot_distance = ranges[FRONT_WALL_ANGLE];

    if (front_robot_distance < MIN_DIST_FRONT_WALL) {
      return State::DIVE_LEFT;
    } else if (right_wall_distance < MIN_DIST_RIGHT_WALL) {
      return State::GET_FARTHER;
    } else {
      return State::GET_CLOSER;
    }
  }

  /**
   * @brief Depending on the current state, publish the next movement of the
   * robot
   *
   */
  void perform_action() {
    switch (curr_state_) {
    case State::GET_CLOSER:
        RCLCPP_INFO(this->get_logger(), "GETTING CLOSER");
      publish_vel(LINEAR_VEL, -1 * ANGULAR_VEL);
      break;
    case State::GET_FARTHER:
    RCLCPP_INFO(this->get_logger(), "GETTING FARTHER");
      publish_vel(LINEAR_VEL, ANGULAR_VEL);
      break;
    case State::DIVE_LEFT:
    RCLCPP_INFO(this->get_logger(), "DIVING LEFT");
      publish_vel(LINEAR_VEL, DIVING_VEL);
      break;
    default:
      publish_vel(LINEAR_VEL, NO_ANGULAR_VEL);
      break;
    }
  }

  /**
   * @brief Publish velocity to the robot
   *
   * @param linear_x tells the velocity of the robot moving forward/backward
   * @param angular_z tells the velocity of the rotation of the robot
   */
  void publish_vel(float linear_x, float angular_z) {
    auto msg = TwistRobot();
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;

    pub_vel_->publish(msg);
  }

  /**
   * @brief Maintain the robot at least 0.2m close to the right wall and circle
   * around the track
   *
   * @param message contains information about the environment of the robot
   */
  void scan_callback(const LaserScan::SharedPtr &message) {
    const auto ranges = message->ranges;

    const State new_state = get_state(ranges);
    if (curr_state_ != new_state) {
      curr_state_ = new_state;
      perform_action();
    }
  }

public:
  explicit FollowWall() : Node("client_follow_wall_node") {

    //* Create both publisher and subscriber
    sub_scan_ = this->create_subscription<LaserScan>(
        SUB_NAME, GLOBAL_QOS,
        [this](const LaserScan::SharedPtr msg) { this->scan_callback(msg); });

    pub_vel_ = this->create_publisher<TwistRobot>(PUB_NAME, GLOBAL_QOS);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FollowWall>());
  rclcpp::shutdown();
  return 0;
}