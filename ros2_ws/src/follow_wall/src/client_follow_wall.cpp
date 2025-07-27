#include "follow_wall/action_manager.hpp"
#include "follow_wall/common_types.hpp"
#include "follow_wall/movement_controller.hpp"
#include "follow_wall/scan_processor.hpp"
#include <atomic>
#include <chrono>
#include <custom_messages/action/detail/odom_record__struct.hpp>
#include <custom_messages/srv/detail/find_wall__struct.hpp>
#include <experimental/string_view>
#include <future>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <memory>
#include <rclcpp/callback_group.hpp>
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
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/create_client.hpp>
#include <rcutils/logging.h>
#include <rmw/types.h>
#include <sensor_msgs/msg/detail/laser_scan__struct.hpp>
#include <string>

class FollowWall : public rclcpp::Node {
private:
  //* Constants on program
  static constexpr const char *SUB_NAME = "scan";
  static constexpr const char *PUB_NAME = "cmd_vel";
  static constexpr int GLOBAL_QOS = 10;
  static constexpr const char *SERVICE_NAME = "position_robot";
  static constexpr const char *SRVR_ACTION_NAME = "record_odom";

  //* Shortening names
  using LaserScan = sensor_msgs::msg::LaserScan;
  using TwistMsg = geometry_msgs::msg::Twist;
  using FindWall = custom_messages::srv::FindWall;
  using OdomMsg = custom_messages::action::OdomRecord;
  using State = FollowWallTypes::State;

  State curr_state_{State::IDLE};
  std::atomic<bool> goal_done_;

  //* Subscriber, publishers, clients
  rclcpp::Subscription<LaserScan>::SharedPtr sub_scan_;
  rclcpp::Publisher<TwistMsg>::SharedPtr pub_vel_;
  rclcpp_action::Client<OdomMsg>::SharedPtr act_odom_;
  rclcpp::CallbackGroup::SharedPtr act_odom_group_;

  //* Helper functions
  std::unique_ptr<ScanProcessor> scan_processor_;
  std::unique_ptr<MovementController> movement_controller_;
  std::unique_ptr<ActionManager> action_manager_;
  rclcpp::Logger own_logger_ = this->get_logger();

  /**
   * @brief Maintain the robot at least 0.2m close to the right wall and circle
   * around the track
   *
   * @param message contains information about the environment of the robot
   */
  void scan_callback(const LaserScan::SharedPtr &message) {
    const auto ranges = message->ranges;

    const State new_state = scan_processor_->determine_state(ranges);
    if (curr_state_ != new_state) {
      curr_state_ = new_state;
      movement_controller_->perform_action(curr_state_);
    }
  }

  void initialize_publisher() {
    //* Need to initialize movement_controller when pub_vel is available
    pub_vel_ = this->create_publisher<TwistMsg>(PUB_NAME, GLOBAL_QOS);
    movement_controller_ =
        std::make_unique<MovementController>(own_logger_, pub_vel_);
  }

  void initialize_subscriber() {
    //* Create both publisher and subscriber
    sub_scan_ = this->create_subscription<LaserScan>(
        SUB_NAME, GLOBAL_QOS,
        [this](const LaserScan::SharedPtr msg) { this->scan_callback(msg); });
  }

  void initialize_action() {
    act_odom_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    act_odom_ = rclcpp_action::create_client<OdomMsg>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), SRVR_ACTION_NAME,
        act_odom_group_);

    //* Lambda function that when the action finishes, needs to be executed
    auto finish_execution = [this]() {
      this->goal_done_ = false;
      this->movement_controller_->stop_robot();
    };

    action_manager_ = std::make_unique<ActionManager>(own_logger_, act_odom_,
                                                      finish_execution);

    action_manager_->start_action_async();
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
      RCLCPP_ERROR(own_logger_,
                   "Failed to receive LocalizePart service response");
      return;
    }
    executor.remove_node(this->get_node_base_interface());

    auto response = future.get();
    if (!response->wallfound) {
      RCLCPP_ERROR(own_logger_, "Service failed");
      return;
    }
  }

public:
  explicit FollowWall() : Node("client_follow_wall_node"), goal_done_(false) {
    scan_processor_ = std::make_unique<ScanProcessor>(own_logger_);

    //* Call first the service, which then will initialize the subscriber and
    // perform the action
    call_service();
    initialize_subscriber();
    initialize_publisher();
    initialize_action();
  }

  bool is_goal_done() { return goal_done_; }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto client = std::make_shared<FollowWall>();

  //* Set the logs of this node and the distance_tracker
  auto _ = rcutils_logging_set_logger_level(client->get_logger().get_name(),
                                            RCUTILS_LOG_SEVERITY_INFO);
  (void)_;

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(client);

  while (!client->is_goal_done() && rclcpp::ok()) {
    executor.spin_some();
  }

  rclcpp::shutdown();
  return 0;
}