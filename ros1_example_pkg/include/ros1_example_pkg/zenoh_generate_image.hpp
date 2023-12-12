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
  // ros::Publisher image_pub_;

  zenohc::Session* z_session_;
  // get this error if z_manager_ is uncommented:
  //  error: use of deleted function ‘zenohc::ShmManager::ShmManager()
  //    [inherited from zenohcxx::Owned<zc_owned_shm_manager_t>]’
  zenohc::ShmManager* z_manager_;
  zenohc::Publisher z_pub_;

  BouncingBall bouncing_ball_;
};
