/*
 * Copyright 2023 Lucas Walter
 *
 * Generate an image and publish it in ros
 */

#include <utility>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <zenoh.hxx>

using zenoh::Publisher;
using zenoh::PublisherPutOptions;
using zenoh::Session;
using zenoh::ShmManager;
using zenoh::expect;


void update_value(float& value, float& velocity,
                  float min_value, float max_value, const float margin = 0)
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

class BouncingBall
{
public:
  void update(cv::Mat& image)
  {
    image = cv::Mat(cv::Size(width_, height_), CV_8UC3, cv::Scalar::all(0));
    cv::circle(image, cv::Point2f(x_, y_), radius_,
               cv::Scalar(200, 190, 180), -1);
    update_value(x_, vel_x_, 0, width_, radius_);
    update_value(y_, vel_y_, 0, height_, radius_);
  }

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

class GenerateImage
{
public:
  GenerateImage(Session* z_session, ShmManager* z_manager) :
    private_nh_("~"),
    z_session_(z_session),
    z_manager_(z_manager),
    z_pub_(expect<Publisher>(z_session->declare_publisher("image")))
  {
    // image_pub_ = nh_.advertise<sensor_msgs::Image>("image", 4);
    double period = 0.033;
    private_nh_.getParam("period", period);
    timer_ = nh_.createTimer(ros::Duration(period), &GenerateImage::update, this);
  }

  void update(const ros::TimerEvent& event);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Timer timer_;
  // ros::Publisher image_pub_;

  Session* z_session_;
  // get this error if z_manager_ is uncommented:
  //  error: use of deleted function ‘zenohc::ShmManager::ShmManager()
  //    [inherited from zenohcxx::Owned<zc_owned_shm_manager_t>]’
  ShmManager* z_manager_;
  Publisher z_pub_;

  BouncingBall bouncing_ball_;
};
