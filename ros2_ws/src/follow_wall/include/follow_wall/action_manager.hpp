#pragma once

#include <algorithm>
#include <custom_messages/action/detail/odom_record__struct.hpp>
#include <functional>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp_action/client.hpp>
class ActionManager {
private:
  using OdomMsg = custom_messages::action::OdomRecord;
  using GoalHandle = rclcpp_action::ClientGoalHandle<OdomMsg>;

  //* Variables
  rclcpp::Logger logger_;
  rclcpp_action::Client<OdomMsg>::SharedPtr act_odom_;
  std::function<void()> finish_execution_;

  void goal_response_callback(const GoalHandle::SharedPtr &goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(logger_, "Goal was rejected by server");
    } else {
      RCLCPP_INFO(logger_, "Goal accepted by server, waiting for result");
    }
  }

  void
  feedback_callback(GoalHandle::SharedPtr,
                    const std::shared_ptr<const OdomMsg::Feedback> feedback) {
    RCLCPP_INFO(logger_, "Feedback received: %f", feedback->current_total);
  }

  void result_callback(const GoalHandle::WrappedResult &result) {
    finish_execution_();

    switch (result.code) {
    case rclcpp_action::ResultCode::UNKNOWN:
      RCLCPP_ERROR(logger_, "Unknown result code");
      return;
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(logger_, "Number of odoms: %i",
                  result.result->list_of_odoms.size());
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(logger_, "Goal was canceled");
      return;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(logger_, "Goal was aborted");
      return;
    }
  }

public:
  explicit ActionManager(const rclcpp::Logger &logger,
                         rclcpp_action::Client<OdomMsg>::SharedPtr act_odom, std::function<void()>&& finish_execution)
      : logger_(logger), act_odom_(act_odom), finish_execution_(std::move(finish_execution)) {}

  /**
   * @brief Calls the action server to start recording the odometry of the robot
   *
   */
  void start_action_async() {
    if (!this->act_odom_) {
      RCLCPP_ERROR(logger_, "Action client not initialized");
      return;
    }

    if (!this->act_odom_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(logger_, "Action not available");
      finish_execution_();
      return;
    }

    auto goal = OdomMsg::Goal();
    goal.num_laps = 1;

    RCLCPP_DEBUG(logger_, "Sending goal");
    auto send_goal_options = rclcpp_action::Client<OdomMsg>::SendGoalOptions();

    //* Sets the necessary callbacks needed before making requests
    send_goal_options.goal_response_callback =
        [this](std::shared_future<std::shared_ptr<GoalHandle>> goal_handle) {
          this->goal_response_callback(goal_handle.get());
        };

    send_goal_options.feedback_callback =
        [this](GoalHandle::SharedPtr _,
               const std::shared_ptr<const OdomMsg::Feedback> feedback) {
          this->feedback_callback(_, feedback);
        };

    send_goal_options.result_callback =
        [this](const GoalHandle::WrappedResult result) {
          this->result_callback(result);
        };

    auto goal_handle_future =
        this->act_odom_->async_send_goal(goal, send_goal_options);
  }
};