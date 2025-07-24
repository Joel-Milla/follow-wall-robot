#include <chrono>
#include <custom_messages/srv/detail/find_wall__struct.hpp>
#include <experimental/string_view>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <memory>
#include <rclcpp/client.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/future_return_code.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/utilities.hpp>
#include <rcutils/logging.h>
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
  static constexpr const char *SERVICE_NAME = "position_robot";

  //* Shortening names
  using LaserScan = sensor_msgs::msg::LaserScan;
  using TwistMsg = geometry_msgs::msg::Twist;
  using FindWall = custom_messages::srv::FindWall;

  enum State { IDLE, GET_CLOSER, GET_FARTHER, DIVE_LEFT };
  State curr_state_{IDLE};

  //* Subscriber and publishers
  rclcpp::Subscription<LaserScan>::SharedPtr sub_scan_;
  rclcpp::Publisher<TwistMsg>::SharedPtr pub_vel_;

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
      RCLCPP_DEBUG(this->get_logger(), "GETTING CLOSER");
      publish_vel(LINEAR_VEL, -1 * ANGULAR_VEL);
      break;
    case State::GET_FARTHER:
      RCLCPP_DEBUG(this->get_logger(), "GETTING FARTHER");
      publish_vel(LINEAR_VEL, ANGULAR_VEL);
      break;
    case State::DIVE_LEFT:
      RCLCPP_DEBUG(this->get_logger(), "DIVING LEFT");
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
    auto msg = TwistMsg();
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

  void initialize_subscriber_publisher() {
    //* Create both publisher and subscriber
    sub_scan_ = this->create_subscription<LaserScan>(
        SUB_NAME, GLOBAL_QOS,
        [this](const LaserScan::SharedPtr msg) { this->scan_callback(msg); });

    pub_vel_ = this->create_publisher<TwistMsg>(PUB_NAME, GLOBAL_QOS);
  }

  /**
   * @brief This function calls the service in charge of finding the wall and
   * positioning the robot parallel to it.
   *
   */
  void call_service() {
    rclcpp::Client<FindWall>::SharedPtr client =
        this->create_client<FindWall>(SERVICE_NAME);

    //* Check that the service is available
    if (!client->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Unable to find service.");
      return;
    }

    //* Send request
    auto request = std::make_shared<FindWall::Request>();
    auto future = client->async_send_request(request);

    //* The executor allows to use resources to wait for the service to finish
    // and then be able to initialize the subscribers and publishers
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(this->get_node_base_interface());
    if (executor.spin_until_future_complete(future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to receive LocalizePart service response");
      return;
    }
    executor.remove_node(this->get_node_base_interface());

    auto response = future.get();
    if (!response->wallfound) {
      RCLCPP_ERROR(this->get_logger(), "Service failed");
      return;
    }

    initialize_subscriber_publisher();
  }

public:
  explicit FollowWall() : Node("client_follow_wall_node") {
    //* Call first the service, which then will initialize the subscriber and
    // perform the action
    call_service();
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto client = std::make_shared<FollowWall>();

  //* Set the logs of this node and the distance_tracker
  auto _ = rcutils_logging_set_logger_level(client->get_logger().get_name(),
                                            RCUTILS_LOG_SEVERITY_DEBUG);
  (void)_;

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(client);
  executor.spin();
  rclcpp::spin(std::make_shared<FollowWall>());

  rclcpp::shutdown();
  return 0;
}