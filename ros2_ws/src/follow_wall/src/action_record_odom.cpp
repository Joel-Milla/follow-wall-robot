#include <cmath>
#include <custom_messages/action/detail/odom_record__struct.hpp>
#include <geometry_msgs/msg/detail/point32__struct.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos_event.hpp>
#include <rclcpp/subscription_options.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp_action/types.hpp>
#include <thread>
#include <vector>

class DistanceTracker {
private:
  static constexpr float PROXIMITY_THRESHOLD = 0.1f;
  static constexpr float LEAVE_THRESHOLD = 0.3f;

  using Point = geometry_msgs::msg::Point32;

  std::mutex mutex_;
  std::vector<Point> checkpoints;
  Point current_pos;
  int completed_laps_;
  rclcpp::Logger logger = rclcpp::get_logger("distance_tracker");
  bool near_start_position_ = false;

  float distance_between(const Point &point1, const Point &point2) {
    float difference_x = (point1.x - point2.x);
    float difference_y = (point1.y - point2.y);

    return std::hypot(difference_x, difference_y);
  }

public:
  //* Reset all the variables with the default values
  void reset() {
    completed_laps_ = 0;
    target_laps = 0;
    has_received_odom_message = false;
    checkpoints = std::vector<Point>();
  }

  bool has_received_odom_message; //* When true, can start creating checkpoints
  int target_laps;
  /**
   * @brief Function called by the subscriber of main class that updates the
   * current odometry of robot
   *
   * @param new_x
   * @param new_y
   * @param new_theta
   */
  void update_position(float new_x, float new_y, float new_theta) {
    std::lock_guard<std::mutex> lock{mutex_};

    //* After this is set to true, it means that the current_pos has a valid
    // position, and will save it
    has_received_odom_message = true;
    current_pos.set__x(new_x);
    current_pos.set__y(new_y);
    current_pos.set__z(new_theta);
  }

  /**
   * @brief Save the current position of the robot in the vector 'checkpoints'
   * that will be called from the action server every 'HZ_FREQUENCY'
   *
   * @return float of the traveled distance between the last known checkpoint
   * and the current position of the robot
   */
  float create_checkpoint() {
    std::lock_guard<std::mutex> lock{mutex_};

    if (!has_received_odom_message)
      return 0.0f;

    size_t num_checkpoints = checkpoints.size();
    if (checkpoints.empty()) {
      checkpoints.push_back(current_pos);
      return 0.0f;
    }

    Point last_checkpoint = checkpoints[num_checkpoints - 1];
    float distance_traveled = distance_between(last_checkpoint, current_pos);
    RCLCPP_DEBUG(logger, "Distance between (%f, %f) and (%f, %f) is: %f",
                 last_checkpoint.x, last_checkpoint.y, current_pos.x,
                 current_pos.y, distance_traveled);

    checkpoints.push_back(current_pos);

    return distance_traveled;
  }

  /**
   * @brief Checks if the robot is near its starting location. If true, means
   * that has completed a lap, so update completed_laps_ variable
   *
   */
  void update_laps() {
    if (checkpoints.size() < 2)
      return;

    const Point &first_pos = checkpoints[0];
    const Point &last_pos = checkpoints[checkpoints.size() - 1];
    float distance = distance_between(first_pos, last_pos);

    if (!near_start_position_ && distance <= PROXIMITY_THRESHOLD) {
      completed_laps_++;
      near_start_position_ = true;
      RCLCPP_DEBUG(logger, "Updated laps to: %i", completed_laps_);
    } else if (near_start_position_ && distance > LEAVE_THRESHOLD) {
      near_start_position_ = false;
      RCLCPP_DEBUG(logger, "Robot left the area");
    }
  }

  bool is_finished() {
    std::lock_guard<std::mutex> lock{mutex_};
    return target_laps == completed_laps_;
  }

  std::vector<Point> get_checkpoints() { return checkpoints; }
};

class OdomTracker : public rclcpp::Node {
private:
  //* To improve readability
  using OdomMsg = nav_msgs::msg::Odometry;
  using OdomRecord = custom_messages::action::OdomRecord;
  using GoalHandle = rclcpp_action::ServerGoalHandle<OdomRecord>;

  //* Constatns
  static constexpr const char *SUB_NAME = "odom";
  static constexpr const char *SRV_ACTION_NAME = "record_odom";
  static constexpr int GENERAL_QOS = 10;
  static constexpr float HZ_FREQUENCY = (1.0f / 2.0f); //* Updates per second

  //* RCLCPP varaibles
  rclcpp_action::Server<OdomRecord>::SharedPtr srv_odom_;
  rclcpp::Subscription<OdomMsg>::SharedPtr sub_odom_;
  rclcpp::CallbackGroup::SharedPtr sub_group;

  //* State managed variables
  DistanceTracker tracker_;
  enum State { IDLE, PROCESSING };
  State curr_state{IDLE};

  /**
   * @brief Receives the message from the topic 'SUB_NAME' and updates the
   * current position of the robot
   *
   * @param msg
   */
  void subscriber_callback(const OdomMsg::SharedPtr &msg) {
    if (curr_state == IDLE)
      return;

    auto point = msg->pose.pose.position;
    tracker_.update_position(point.x, point.y, point.z);
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const OdomRecord::Goal> &goal) {
    RCLCPP_DEBUG(this->get_logger(), "Received goal with laps: %i",
                 goal->num_laps);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandle> &goal_handle) {
    RCLCPP_DEBUG(this->get_logger(), "Received cancel request");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  /**
   * @brief Function that receives the goal after server accepted, and then
   * creates separate thread so the execution can run on its own and doesnt
   * block the current threads
   *
   * @param goal_handle
   */
  void handle_accepted(const std::shared_ptr<GoalHandle> &goal_handle) {
    std::thread([this, goal_handle]() { this->execute(goal_handle); }).detach();
  }

  //* The next two functions reset the state management variables before/after
  // the action server execution
  void reset_before_execution() {
    curr_state = PROCESSING;
    tracker_.reset();
  }

  void reset_after_execution() { curr_state = IDLE; }

  void execute(const std::shared_ptr<GoalHandle> &goal_handle) {
    RCLCPP_DEBUG(this->get_logger(), "Starting main execution");

    reset_before_execution();
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<OdomRecord::Feedback>();
    auto result = std::make_shared<OdomRecord::Result>();
    rclcpp::Rate loop_rate{HZ_FREQUENCY};

    int number_laps = goal->num_laps;
    tracker_.target_laps = number_laps;
    float &distance_traveled = feedback->current_total;

    //* While loop runs every HZ_FREQUENCY per second. Each time, it will create
    // a 'checkpoint', saving the current position of the robot, and updating
    // the total distance traveled by it
    while (!tracker_.is_finished() && rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        result->list_of_odoms = tracker_.get_checkpoints();
        goal_handle->canceled(result);
        RCLCPP_DEBUG(this->get_logger(), "Goal canceled");
        return;
      }

      distance_traveled += tracker_.create_checkpoint();
      tracker_.update_laps();

      goal_handle->publish_feedback(feedback);
      RCLCPP_DEBUG(this->get_logger(), "Feedback published");

      loop_rate.sleep();
    }

    if (rclcpp::ok()) {
      result->list_of_odoms = tracker_.get_checkpoints();
      goal_handle->succeed(result);
      RCLCPP_DEBUG(this->get_logger(), "Goal succeed");
    }

    RCLCPP_DEBUG(this->get_logger(), "Finished execution");
    reset_after_execution();
  }

public:
  explicit OdomTracker(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("action_record_odom_node", options) {

    //* Create subscription with its callback groupt
    sub_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = sub_group;
    sub_odom_ = this->create_subscription<OdomMsg>(
        SUB_NAME, GENERAL_QOS,
        [this](const OdomMsg::SharedPtr msg) { subscriber_callback(msg); },
        sub_options);

    //* Create action
    this->srv_odom_ = rclcpp_action::create_server<OdomRecord>(
        this, SRV_ACTION_NAME,
        [this](const rclcpp_action::GoalUUID &uuid,
               std::shared_ptr<const OdomRecord::Goal> goal) {
          return this->handle_goal(uuid, goal);
        },
        [this](const std::shared_ptr<GoalHandle> &goal_handle) {
          return this->handle_cancel(goal_handle);
        },
        [this](const std::shared_ptr<GoalHandle> &goal_handle) {
          this->handle_accepted(goal_handle);
        });
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto action_srv = std::make_shared<OdomTracker>();
  //* Set the logs of this node and the distance_tracker
  auto _ = rcutils_logging_set_logger_level(action_srv->get_logger().get_name(),
                                            RCUTILS_LOG_SEVERITY_DEBUG);
  (void)_;
  auto __ = rcutils_logging_set_logger_level("distance_tracker",
                                             RCUTILS_LOG_SEVERITY_DEBUG);
  (void)_, (void)__;

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_srv);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}