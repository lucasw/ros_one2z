/*
 * Copyright 2023 Lucas Walter
 *
 * Generate an image and publish it in ros
 */

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>


class BouncingBall
{
public:
  void update(cv::Mat& image)
  {
    image = cv::Mat(cv::Size(width_, height_), CV_8UC3, cv::Scalar::all(0));
    // TODO(lucasw) draw a bouncing ball
  }

private:
  size_t width_ = 2048;
  size_t height_ = 1024;
};

class GenerateImage
{
public:
  GenerateImage() : private_nh_("~")
  {
    image_pub_ = nh_.advertise<sensor_msgs::Image>("image", 4);

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
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Timer timer_;
  ros::Publisher image_pub_;

  BouncingBall bouncing_ball_;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "generate_image");
  GenerateImage generate_image;
  ros::spin();
}
