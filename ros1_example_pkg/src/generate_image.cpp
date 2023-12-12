/*
 * Copyright 2023 Lucas Walter
 *
 * Generate an image and publish it in ros
 */

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>

// TODO(lucasw) supposed to be transparent whether ros_comm is using zenoh,
// shouldn't leak into this and every C++, currently without this this node
// won't link when using the zenoh ros_comm- maybe could hide something in a header?
#if 0
namespace zenohc {
using namespace zenohcxx;
#include <zenohcxx/impl.hxx>
}
#endif

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
  GenerateImage() : private_nh_("~")
  {
    image_pub_ = nh_.advertise<sensor_msgs::Image>("image", 4);
    test_pub_ = nh_.advertise<std_msgs::Float64>("test", 4);

    double period = 0.033;
    private_nh_.getParam("period", period);
    timer_ = nh_.createTimer(ros::Duration(period), &GenerateImage::update, this);
  }

  void update(const ros::TimerEvent& event)
  {
    cv_bridge::CvImage cv_image;
    bouncing_ball_.update(cv_image.image);
    auto image_msg = cv_image.toImageMsg();
    image_msg->header.stamp = event.current_real;
    image_msg->encoding = "rgb8";
    image_pub_.publish(image_msg);

    auto float_msg = std_msgs::Float64();
    float_msg.data = 0.12345689101112131415;
    test_pub_.publish(float_msg);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Timer timer_;
  ros::Publisher image_pub_;
  ros::Publisher test_pub_;

  BouncingBall bouncing_ball_;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "generate_image");
  GenerateImage generate_image;
  ros::spin();
}
