/*
 * Copyright 2023 Lucas Walter
 *
 * Generate an image and publish it in ros
 */

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>

#include <ros1_example_pkg/bouncing_ball.hpp>

// TODO(lucasw) supposed to be transparent whether ros_comm is using zenoh,
// shouldn't leak into this and every C++, currently without this this node
// won't link when using the zenoh ros_comm- maybe could hide something in a header?
#if 0
namespace zenohc {
using namespace zenohcxx;
#include <zenohcxx/impl.hxx>
}
#endif


class GenerateImage
{
public:
  GenerateImage() : private_nh_("~")
  {
    image_pub_ = nh_.advertise<sensor_msgs::Image>("image", 4);
    test0_pub_ = nh_.advertise<std_msgs::Float64>("test0", 4);
    test1_pub_ = nh_.advertise<std_msgs::Float64>("test1", 4);

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

    // TODO(lucasw) commingling this with image_pub publish using zenoh ros_comm is crashing
    auto float_msg = std_msgs::Float64();
    float_msg.data = 0.12345689101112131415;
    test0_pub_.publish(boost::make_shared<std_msgs::Float64>(float_msg));
    float_msg.data = 333.0333;
    test1_pub_.publish(float_msg);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Timer timer_;
  ros::Publisher image_pub_;
  ros::Publisher test0_pub_;
  ros::Publisher test1_pub_;

  BouncingBall bouncing_ball_;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "generate_image");
  GenerateImage generate_image;
  ros::spin();
}
