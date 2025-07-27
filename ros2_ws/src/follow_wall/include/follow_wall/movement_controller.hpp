#pragma once

#include "follow_wall/common_types.hpp"
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/publisher.hpp>

/**
 * @brief Handles robot movement commands
 *
 */
class MovementController {
private:
  using State = FollowWallTypes::State;
  using TwistMsg = geometry_msgs::msg::Twist;

  static inline constexpr float LINEAR_VEL = 0.14f;
  static inline constexpr float ANGULAR_VEL = 0.46f;
  static inline constexpr float DIVING_VEL = 1.2f;
  static inline constexpr float NO_VEL = 0.0f;

  const rclcpp::Logger logger_;
  rclcpp::Publisher<TwistMsg>::SharedPtr pub_vel_;

public:
  /**
   * @brief Depending on the current state, publish the next movement of the
   * robot
   *
   */
  void perform_action(const State &curr_state_) {
    switch (curr_state_) {
    case State::STOP:
      RCLCPP_DEBUG(logger_, "STOPPING");
      publish_vel(NO_VEL, NO_VEL);
      break;
    case State::GET_CLOSER:
      RCLCPP_DEBUG(logger_, "GETTING CLOSER");
      publish_vel(LINEAR_VEL, -1 * ANGULAR_VEL);
      break;
    case State::GET_FARTHER:
      RCLCPP_DEBUG(logger_, "GETTING FARTHER");
      publish_vel(LINEAR_VEL, ANGULAR_VEL);
      break;
    case State::DIVE_LEFT:
      RCLCPP_DEBUG(logger_, "DIVING LEFT");
      publish_vel(LINEAR_VEL, DIVING_VEL);
      break;
    default:
      publish_vel(LINEAR_VEL, NO_VEL);
      break;
    }
  }

  void stop_robot() { publish_vel(NO_VEL, NO_VEL); }

  /**
   * @brief Publish velocity to the robot
   *
   * @param linear_x tells the velocity of the robot moving forward/backward
   * @param angular_z tells the velocity of the rotation of the robot
   */
  void publish_vel(const float &linear_x, const float &angular_z) {
    auto msg = TwistMsg();
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;

    pub_vel_->publish(msg);
  }

  explicit MovementController(const rclcpp::Logger logger,
                              rclcpp::Publisher<TwistMsg>::SharedPtr pub_vel)
      : logger_(logger), pub_vel_(pub_vel) {}
};