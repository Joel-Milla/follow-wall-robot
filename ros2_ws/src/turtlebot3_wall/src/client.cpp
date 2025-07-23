#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/future_return_code.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_action/client_goal_handle.hpp"
#include "rclcpp_action/create_client.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "turtlebot3_msgs/action/detail/odom_record__struct.hpp"
#include "turtlebot3_msgs/action/odom_record.hpp"
#include "turtlebot3_msgs/srv/find_wall.hpp"
#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <vector>

using FindWallMessage = turtlebot3_msgs::srv::FindWall;
using PointList = turtlebot3_msgs::action::OdomRecord;
using GoalHandleRecord = rclcpp_action::ClientGoalHandle<PointList>;

class FollowWall : public rclcpp::Node {
private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::CallbackGroup::SharedPtr scan_sub_group;

  rclcpp_action::Client<PointList>::SharedPtr client_action_;

  bool goal_done_;

  //* Constants used on all the program
  static constexpr float RIGHT_DISTANCE_LIMIT = 0.2f;
  static constexpr float FRONT_DISTANCE_LIMIT = 0.5f;
  static constexpr float LINEAR_VELOCITY = 0.07f;
  static constexpr float ANGULAR_VELOCITY = 0.1f;
  static constexpr float DIVING_ANGULAR_VELOCITY = 0.6f;
  static constexpr int DEGREE_RIGHT_SIDE = 260;
  static constexpr int DEGREE_FRONT_SIDE = 0;
  static constexpr int QUEUE_SZ = 10;

  enum class State { IDLE, DIVE_LEFT, FORWARD, TURNING_LEFT, TURNING_RIGHT };

  State current_state_{State::IDLE};

  void sensor_receiver(sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
    auto ranges = scan_msg->ranges;

    State next_state = get_next_state(ranges);

    execute_state_machine(next_state);

    if (next_state != current_state_) {
      RCLCPP_INFO(this->get_logger(), "State transition: %s -> %s",
                  state_to_string(next_state).c_str(),
                  state_to_string(next_state).c_str());
      current_state_ = next_state;
    }
  }

  /**
   * @brief Execute the given state
   * @param state to perform
   */
  void execute_state_machine(State state) {
    switch (state) {
    case State::DIVE_LEFT:
      publish_velocity(LINEAR_VELOCITY, DIVING_ANGULAR_VELOCITY);
      break;
    case State::TURNING_RIGHT:
      publish_velocity(LINEAR_VELOCITY, -1 * ANGULAR_VELOCITY);
      break;
    case State::TURNING_LEFT:
      publish_velocity(LINEAR_VELOCITY, ANGULAR_VELOCITY);
      break;
    case State::FORWARD:
      publish_velocity(LINEAR_VELOCITY, 0);
      break;
    case State::IDLE:
      break;
    }
  }

  /**
   * @brief Giving the latest scan, get what is the next state of the robot
   * @param ranges of all the scans
   * @return the next state of the robot
   */
  State get_next_state(std::vector<float> ranges) {
    // index 270 is the degree 270. where the front is given in angle zero
    // (index zero) and the laser scanning the right side is in degree 270 (as
    // laser scan is in unit cirle counter clockwise)
    float right_depth = ranges[DEGREE_RIGHT_SIDE];
    float front_depth = ranges[DEGREE_FRONT_SIDE];

    if (front_depth <= FRONT_DISTANCE_LIMIT) {
      // when the wall is too closed to the front of the robot
      return State::DIVE_LEFT;

    } else if (right_depth > RIGHT_DISTANCE_LIMIT) {
      // If the wall is too far, get close
      return State::TURNING_RIGHT;

    } else if (right_depth < RIGHT_DISTANCE_LIMIT) {
      // If the wall is too close, get far
      return State::TURNING_LEFT;

    } else {
      return State::FORWARD;
    }
  }

  void publish_velocity(float linear_vel, float angular_vel) {
    auto next_move = geometry_msgs::msg::Twist();
    next_move.linear.x = linear_vel;
    next_move.angular.z = angular_vel;
    cmd_vel_pub_->publish(next_move);
  }

  std::string state_to_string(State state) {
    switch (state) {
    case State::IDLE:
      return "IDLE";
    case State::DIVE_LEFT:
      return "DIVE_LEFT";
    case State::FORWARD:
      return "FORWARDS";
    case State::TURNING_LEFT:
      return "TURNING_LEFT";
    case State::TURNING_RIGHT:
      return "TURNING_RIGHT";
    default:
      return "UNKNOWN";
    }
  }

  /**
   * @brief Calls the service 'find_wall' that find the nearest wall, positions
   * the robot at 0.3m, and then rotates so the right side of the robot is
   * parallel to the wall
   */
  void call_find_wall() {
    rclcpp::Client<FindWallMessage>::SharedPtr client =
        this->create_client<FindWallMessage>("find_wall");

    auto request = std::make_shared<FindWallMessage::Request>();

    while (!client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for service");
        return;
      } else {
        RCLCPP_INFO(this->get_logger(), "Service not available");
      }
    }

    auto result_future = client->async_send_request(request);

    // wait for result
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           result_future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto result = result_future.get();
    }
    RCLCPP_INFO(this->get_logger(), "Service finished successfully");
  }

  void goal_response_callback(const GoalHandleRecord::SharedPtr &goal_handle) {
    if (!goal_handle)
      RCLCPP_ERROR(this->get_logger(), "Goal was not accepted");
    else
      RCLCPP_ERROR(this->get_logger(), "Goal was  accepted");
  }

  void
  feedback_callback(const GoalHandleRecord::SharedPtr,
                    const std::shared_ptr<const PointList::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Current distance traveled: %f",
                feedback->current_total);
  }

  void result_callback(const GoalHandleRecord::WrappedResult &result) {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      break;
    case rclcpp_action::ResultCode::UNKNOWN:
      RCLCPP_ERROR(this->get_logger(), "Result unkown");
      break;
    }
    this->goal_done_ = true;
    RCLCPP_INFO(this->get_logger(), "Number of checkpoints received: %f",
                result.result->list_of_odoms.size());
  }

  void send_goal() {
    if (!this->client_action_) {
      RCLCPP_ERROR(this->get_logger(), "Client not initialized");
    }

    if (!this->client_action_->wait_for_action_server(
            std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Server not ready");
      this->goal_done_ = true;
      return;
    }

    auto goal = PointList::Goal();
    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto goal_options = rclcpp_action::Client<PointList>::SendGoalOptions();
    goal_options.goal_response_callback =
        [this](std::shared_future<GoalHandleRecord::SharedPtr> future) {
          auto goal_handle = future.get();
          this->goal_response_callback(goal_handle);
        };

    goal_options.feedback_callback =
        [this](const GoalHandleRecord::SharedPtr goal_handle,
               const std::shared_ptr<const PointList::Feedback> feedback) {
          this->feedback_callback(goal_handle, feedback);
        };

    goal_options.result_callback =
        [this](const GoalHandleRecord::WrappedResult &result) {
          this->result_callback(result);
        };

    auto goal_handle_future =
        this->client_action_->async_send_goal(goal, goal_options);
  }

public:
  explicit FollowWall(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("follow_wall_node", options), goal_done_(false) {
    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    call_find_wall();

    scan_sub_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions scan_options;
    scan_options.callback_group = scan_sub_group;

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", QUEUE_SZ,
        [this](sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
          this->sensor_receiver(scan_msg);
        },
        scan_options);

    this->client_action_ = rclcpp_action::create_client<PointList>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "record_odom");
  }

  bool is_goal_done() const { return this->goal_done_; }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto client_node = std::make_shared<FollowWall>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(client_node);

  while (rclcpp::ok() && !client_node->is_goal_done()) {
    executor.spin_some();
  }

  rclcpp::shutdown();

  return 0;
}