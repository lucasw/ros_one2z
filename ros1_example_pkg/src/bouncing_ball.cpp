/*
 * Copyright 2023 Lucas Walter
 *
 * Generate an image and publish it in ros
 */

#include <utility>

#include <cv_bridge/cv_bridge.h>

#include <ros1_example_pkg/bouncing_ball.hpp>

#include <zenohc.hxx>

void update_value(float& value, float& velocity,
                  float min_value, float max_value, const float margin)
{
  // bounce when reaching limit
  value += velocity;

  min_value += margin;
  if (value <= min_value) {
    value = min_value;
    velocity *= -1;
  }

  max_value -= margin;
  if (value >= max_value) {
    value = max_value;
    velocity *= -1;
  }
}

void BouncingBall::update(cv::Mat& image)
{
  image = cv::Mat(cv::Size(width_, height_), CV_8UC3, cv::Scalar::all(0));
  cv::circle(image, cv::Point2f(x_, y_), radius_,
             cv::Scalar(200, 190, 180), -1);
  update_value(x_, vel_x_, 0, width_, radius_);
  update_value(y_, vel_y_, 0, height_, radius_);
}
