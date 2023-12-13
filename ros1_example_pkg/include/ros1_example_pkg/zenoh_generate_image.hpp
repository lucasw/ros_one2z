/*
 * Copyright 2023 Lucas Walter
 *
 * Generate an image and publish it in ros
 */

#include <utility>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <ros1_example_pkg/bouncing_ball.hpp>

#include <zenohc.hxx>
#if 0
namespace zenohc {
  class Publisher;
  class Session;
  class ShmManager;
}
#endif

class GenerateImage
{
public:
  GenerateImage(zenohc::Session* z_session, zenohc::ShmManager* z_manager);

  void update(const ros::TimerEvent& event);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Timer timer_;

  template <typename M> zenohc::Payload
    ros_msg_to_payload(const boost::shared_ptr<M>& msg);

  // TODO(lucasw) std::shared_ptr
  zenohc::Session* z_session_;
  zenohc::ShmManager* z_manager_;
  zenohc::Publisher z_pub_;
  zenohc::Publisher test0_pub_;
  zenohc::Publisher test1_pub_;

  BouncingBall bouncing_ball_;
};
