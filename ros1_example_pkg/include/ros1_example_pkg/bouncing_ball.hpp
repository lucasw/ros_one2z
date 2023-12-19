/*
 * Copyright 2023 Lucas Walter
 */

#ifndef ROS1_EXAMPLE_PKG_BOUNCING_BALL_HPP
#define ROS1_EXAMPLE_PKG_BOUNCING_BALL_HPP

#include <utility>

#include <cv_bridge/cv_bridge.h>

void update_value(float& value, float& velocity,
                  float min_value, float max_value, const float margin = 0);

class BouncingBall
{
public:
  void update(cv::Mat& image);

  size_t getWidth()
  {
    return width_;
  }

  size_t getHeight()
  {
    return height_;
  }

private:
  size_t width_ = 2048;
  size_t height_ = 1024;
  size_t radius_ = 64;
  float x_ = 100.0;
  float y_ = 120.0;
  float vel_x_ = 14.0;
  float vel_y_ = 30.0;
};

#endif  // ROS1_EXAMPLE_PKG_BOUNCING_BALL_HPP
