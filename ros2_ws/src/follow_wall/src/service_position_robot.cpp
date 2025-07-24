#include <custom_messages/srv/detail/find_wall__struct.hpp>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <iterator>
#include <memory>
#include <mutex>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription_options.hpp>
#include <rclcpp/utilities.hpp>
#include <rcutils/logging.h>
#include <sensor_msgs/msg/detail/laser_scan__struct.hpp>
#include <vector>

enum State { IDLE, FINDING_WALL, MOVING_STRAIGHT, ALIGNING, FINISHED };

struct DistanceTracker {
private:
  static constexpr int NUMBER_ANGLES = 360;
  static constexpr int ANGLE_TOLERANCE = 10;
  static constexpr int FRONT_ANGLE = 0;
  static constexpr int RIGHT_ANGLE = 269;
  static constexpr float PROXIMITY_THRESHOLD = 0.25f;

  std::mutex mutex_;
  std::vector<float> ranges_ =
      std::vector<float>(360, 0); //* Subscriber modified

  //* State variables
  float min_dist_{0.0f};
  int indx_min_dist_;

  /**
   * @brief Based on the current ranges of the robot, update the state variables
   *
   */
  void update_state_variables() {
    RCLCPP_DEBUG(rclcpp::get_logger("distance_tracker"),
                 "Entered update_state_variables");
    std::vector<float>::iterator it =
        std::min_element(std::begin(ranges_), std::end(ranges_));
    indx_min_dist_ = std::distance(std::begin(ranges_), it);
    min_dist_ = ranges_[indx_min_dist_];
    RCLCPP_DEBUG(rclcpp::get_logger("distance_tracker"),
                 "Finished update_state_variables");
  }

  /**
   * @brief Ranges array contains 360 indices, where each index 'i' represent
   * the distance at angle 'i' of the robot. This function, given two angles,
   * check if are close between the PERMITTED_DIST
   *
   * @param angle1
   * @param angle2
   * @return true if the two angles are close enough
   * @return false if the two angles are not close
   */
  bool angles_near(int angle1, int angle2) {
    int upper_bound = (angle1 + ANGLE_TOLERANCE) % NUMBER_ANGLES;
    int lower_bound =
        (((angle1 - ANGLE_TOLERANCE) % NUMBER_ANGLES) + NUMBER_ANGLES) %
        NUMBER_ANGLES; //* This formula prevents negative bounds

    //* Check if the permitted window wraps around 0
    if (lower_bound <= upper_bound) {
      //* Normal case: normal window bounds
      return (angle2 <= upper_bound) && (angle2 >= lower_bound);
    } else {
      //* Wrap around case: window crosses 0, checks if angles are close
      return (angle2 <= upper_bound) || (angle2 >= lower_bound);
    }
  }

public:
  void update_ranges(std::vector<float> &new_ranges) {
    std::lock_guard<std::mutex> lock(mutex_);
    ranges_ = new_ranges;
  }

  /**
   * @brief Based on the current state of the object, tells which action needs
   * to be performed to position the robot in the desired location
   *
   * @return State that needs to be executed
   */
  State get_curr_state() {
    RCLCPP_DEBUG(rclcpp::get_logger("distance_tracker"),
                 "Entered get_curr_state");
    std::lock_guard<std::mutex> lock(mutex_);
    update_state_variables();
    bool within_distance = min_dist_ < PROXIMITY_THRESHOLD;

    if (!within_distance) {
      bool robot_pointing_straight = angles_near(FRONT_ANGLE, indx_min_dist_);
      if (!robot_pointing_straight)
        return State::FINDING_WALL;
      else
        return State::MOVING_STRAIGHT;
    } else {
      bool robot_aligned = angles_near(RIGHT_ANGLE, indx_min_dist_);
      if (!robot_aligned)
        return State::ALIGNING;
      else
        return State::FINISHED;
    }
  }

  bool left_side_closer_to_wall() {
    return indx_min_dist_ < (NUMBER_ANGLES / 2);
  }
};

class PositionRobot : public rclcpp::Node {
private:
  using TwistMsg = geometry_msgs::msg::Twist;
  using LaserScan = sensor_msgs::msg::LaserScan;
  using FindWall = custom_messages::srv::FindWall;

  static constexpr const char *PUBLISHER_TOPIC_NAME = "cmd_vel";
  static constexpr const char *SUBSCRIBER_TOPIC_NAME = "scan";
  static constexpr const char *SERVICE_NAME = "position_robot";
  static constexpr int GENERAL_QOS = 10;
  static constexpr float LINEAR_VEL = 0.1;
  static constexpr float ANGULAR_VEL = 0.3;
  static constexpr float NO_VEL = 0;

  //* Create publishers, subscriber, and service
  rclcpp::Publisher<TwistMsg>::SharedPtr pub_vel_;
  rclcpp::Subscription<LaserScan>::SharedPtr sub_scan_;
  rclcpp::Service<FindWall>::SharedPtr srv_wall_;

  //* Callback groups for multithreading
  rclcpp::CallbackGroup::SharedPtr sub_group;

  //* Variables to manage robot actions
  State curr_state_{IDLE};
  DistanceTracker tracker_;

  /**
   * @brief Update the 360 degree ranges of the current position of the robot
   *
   * @param msg of the scan topic
   */
  void subscriber_callback(const LaserScan::SharedPtr &msg) {
    auto ranges = msg->ranges;
    tracker_.update_ranges(ranges);
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

  void stop_robot() { publish_vel(NO_VEL, NO_VEL); }

  void execute(State state) {
    switch (state) {
    case FINDING_WALL: {
      //* Will rotate the robot to the right/left, depending on which side is
      // closer to get the robot to the front of the wall
      bool left_shortest_path = tracker_.left_side_closer_to_wall();
      if (left_shortest_path)
        publish_vel(NO_VEL, ANGULAR_VEL);
      else
        publish_vel(NO_VEL, -1 * ANGULAR_VEL);
      break;
    }
    case MOVING_STRAIGHT:
      publish_vel(LINEAR_VEL, NO_VEL);
      break;
    case ALIGNING:
      publish_vel(NO_VEL, ANGULAR_VEL);
      break;
    case FINISHED:
      stop_robot();
      break;
    case IDLE:
      stop_robot();
      break;
    }
  }

  /**
   * @brief Callback function of the service that positions the robot at
   * PROXIMITY_THRESHOLD of the wall, with the wall being at angle ~90
   *
   * @param request
   * @param response that will be sent back to the client making the request
   */
  void service_callback(std::shared_ptr<FindWall::Request> &request,
                        std::shared_ptr<FindWall::Response> &response) {
    (void)request; //* the request is not used

    //* Perform an action while the robot is not in the final position
    while ((curr_state_ != FINISHED) && rclcpp::ok()) {
      RCLCPP_DEBUG(this->get_logger(), "Entered while loop");
      State new_state = tracker_.get_curr_state();
      RCLCPP_DEBUG(this->get_logger(), "Finished get_curr_state");

      if (new_state != curr_state_) {
        RCLCPP_DEBUG(this->get_logger(), "New State");
        curr_state_ = new_state;
        execute(curr_state_);
      }
    }

    if (curr_state_ == FINISHED)
      response->wallfound = true;
    else
      response->wallfound = false;

    curr_state_ = IDLE;
  }

public:
  explicit PositionRobot() : Node("service_position_robot_node") {
    //* Subscriber callback group
    sub_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options;
    options.callback_group = sub_group;

    //* Initialize ros2 variables
    pub_vel_ =
        this->create_publisher<TwistMsg>(PUBLISHER_TOPIC_NAME, GENERAL_QOS);

    sub_scan_ = this->create_subscription<LaserScan>(
        SUBSCRIBER_TOPIC_NAME, GENERAL_QOS,
        [this](const LaserScan::SharedPtr msg) {
          this->subscriber_callback(msg);
        },
        options);

    srv_wall_ = this->create_service<FindWall>(
        SERVICE_NAME, [this](std::shared_ptr<FindWall::Request> request,
                             std::shared_ptr<FindWall::Response> response) {
          this->service_callback(request, response);
        });
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto service_node = std::make_shared<PositionRobot>();

  //* Set the logs of this node and the distance_tracker
  auto _ = rcutils_logging_set_logger_level(
      service_node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);
  auto __ = rcutils_logging_set_logger_level("distance_tracker",
                                             RCUTILS_LOG_SEVERITY_INFO);
  (void)_, (void)__;

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(service_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}