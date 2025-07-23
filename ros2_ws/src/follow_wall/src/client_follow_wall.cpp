#include <experimental/string_view>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/detail/laser_scan__struct.hpp>
#include <string>

/*



subscriber to laser topics
publisher to cmd_vel



atomic ranges of type float

State state{
    IDLE,GET_CLOSER,GET_FARTHER, DIVE_LEFT
}

function subscription:
    ranges = message->ranges

    obtain_state

    if new_state != current_state:
        current_state = new_state
        perform_action(current_state)

function publish (float linaer_x, float angular_z):
    Twist linear, angual
    publish

function perform action(state):
    switch
    1.
    2.
    3.

*/

class FollowWall : public rclcpp::Node {
private:
  //* Constants on program
  static constexpr const char *SUB_NAME = "scan";
  static constexpr int GLOBAL_QOS = 10;

  //* Shortening names
  using LaserScan = sensor_msgs::msg::LaserScan;
  using TwistRobot = geometry_msgs::msg::Twist;

  enum State { IDLE, GET_CLOSER, GET_FARTHER, DIVE_LEFT };
  State curr_state_;

  //* Subscriber and publishers
  rclcpp::Subscription<LaserScan>::SharedPtr sub_scan_;
  rclcpp::Publisher<TwistRobot>::SharedPtr pub_vel_;

  void scan_callback(const LaserScan::SharedPtr &message) {}

public:
  explicit FollowWall() : Node("client_follow_wall_node") {
    sub_scan_ = this->create_subscription<LaserScan>(
        SUB_NAME, GLOBAL_QOS,
        [this](const LaserScan::SharedPtr msg) { this->scan_callback(msg); });
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::shutdown();
  return 0;
}

/**
subscribe to laser topics
publish vel
*/