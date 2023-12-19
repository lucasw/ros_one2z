/*
 * Copyright 2023 Lucas Walter
 *
 * Generate an image and publish it in ros
 */

#include <utility>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PolygonStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>


class ImageToContour
{
public:
  ImageToContour();

  void imageCallback(const sensor_msgs::ImageConstPtr& image_msg);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber image_sub_;
  ros::Publisher contour_pub_;
};

ImageToContour::ImageToContour() :
    private_nh_("~")
{
  contour_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("contour", 3);
  image_sub_ = nh_.subscribe("image", 3, &ImageToContour::imageCallback, this);
}

void ImageToContour::imageCallback(const sensor_msgs::ImageConstPtr& image_msg)
{
  ROS_INFO_STREAM(image_msg->width << "x" << image_msg->height << " " << image_msg->encoding);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "image_to_contour");
  ImageToContour image_to_contour;
  ros::spin();
}
