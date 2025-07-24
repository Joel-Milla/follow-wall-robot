#include "custom_messages/action/odom_record.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <custom_messages/action/detail/odom_record__struct.hpp>
#include <geometry_msgs/msg/detail/point32__struct.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
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

struct DistanceTracker {
private:
  using Point = geometry_msgs::msg::Point32;

  std::mutex mutex_;
  int num_laps_;
  std::vector<Point> checkpoints;
  Point current_pos;

public:
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

    current_pos.set__x(new_x);
    current_pos.set__y(new_y);
    current_pos.set__z(new_theta);
  }
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
    std::thread([this, &goal_handle]() {
      this->execute(goal_handle);
    }).detach();
  }

  void execute(const std::shared_ptr<GoalHandle> &goal_handle) {
    (void)goal_handle;
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
        [this](const OdomMsg::SharedPtr msg) { subscriber_callback(msg); });

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

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_srv);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}