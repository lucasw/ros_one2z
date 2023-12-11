/*
 * Copyright 2023 Lucas Walter
 *
 * Generate an image and publish it in ros
 */

#include <utility>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <ros1_example_pkg/zenoh_generate_image.hpp>

using zenoh::Encoding;
using zenoh::EncodingPrefix;
using zenoh::PublisherPutOptions;
using zenoh::ShmManager;
using zenoh::Shmbuf;
using zenoh::open;


void GenerateImage::update(const ros::TimerEvent& event)
{
  cv_bridge::CvImage cv_image;
  bouncing_ball_.update(cv_image.image);
  auto image_msg = cv_image.toImageMsg();
  image_msg->header.stamp = event.current_real;
  image_msg->encoding = "rgb8";
  // image_pub_.publish(image_msg);

  PublisherPutOptions options;
  Encoding encoding;
  // TODO(lucasw) is the encoding string sent every single message?
  // would rather set it somewhere like a rosparam for the entire channel
  // (though a competing publisher may send the wrong type),
  // subscriber can get it once and assume all following messages are same type
  encoding.set_prefix(
    EncodingPrefix::Z_ENCODING_PREFIX_APP_OCTET_STREAM).set_suffix(
    "Image");
    // this is too long
    // ros::message_traits::Definition<sensor_msgs::Image>::value());
  ROS_INFO_STREAM_ONCE(encoding.get_suffix().as_string_view());

  options.set_encoding(encoding);

  const auto length = ros::serialization::serializationLength(*image_msg);
  auto shmbuf = expect<Shmbuf>(z_manager_->alloc(length));

  // std::vector<uint8_t> buffer(length);
  // ros::serialization::OStream ostream(&buffer[0], length);
  // ros::serialization::serialize(ostream, float_msg);
  ros::serialization::OStream ostream(shmbuf.ptr(), length);
  ros::serialization::serialize(ostream, *image_msg);

  auto payload = shmbuf.into_payload();
  z_pub_.put_owned(std::move(payload), options);

  ROS_INFO_STREAM_THROTTLE(2.0, "published '" << "Image"
      << "' of length: " << length);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "zenoh_generate_image");
  z_owned_config_t config = z_config_default();
  ROS_INFO_STREAM("opening zeno session");
  auto z_session = expect<Session>(open(std::move(config)));

  ROS_INFO_STREAM("opened zenoh session: " << z_session.info_zid());

  std::ostringstream oss;
  oss << z_session.info_zid();
  // May have to make this larger depending on bouncing ball width and height
  auto z_manager = expect<ShmManager>(shm_manager_new(z_session, oss.str().c_str(),
                                                      2048 * 1024 * 3 * 4));

  GenerateImage generate_image(&z_session, &z_manager);
  ros::spin();
}
