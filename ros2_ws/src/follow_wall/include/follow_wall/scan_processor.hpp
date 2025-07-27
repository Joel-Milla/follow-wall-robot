#pragma once

#include "follow_wall/common_types.hpp"
#include <rclcpp/logger.hpp>
#include <vector>

/**
 * @brief Handles laser scan processing and state determination
 */
class ScanProcessor {
private:
  using State = FollowWallTypes::State;

  static constexpr int RIGHT_WALL_ANGLE = 270;
  static constexpr int FRONT_WALL_ANGLE = 0;
  static constexpr float MIN_DIST_RIGHT_WALL = 0.22;
  static constexpr float MIN_DIST_FRONT_WALL = 0.5;

  rclcpp::Logger logger_;

public:
  explicit ScanProcessor(rclcpp::Logger logger) : logger_(logger) {}

  /**
   * @brief Get the state of the robot based on ranges
   *
   * @param ranges is an array of size 360, where each angle is the distance to
   * an object at that angle
   * @return state of the robot
   */
  State determine_state(const std::vector<float> &ranges) {
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
};