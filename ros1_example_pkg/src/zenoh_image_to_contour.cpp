/*
 * Copyright 2023 Lucas Walter
 *
 * Generate an image and publish it in ros
 */

#include <utility>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>


class ImageToContour
{
public:
  ImageToContour(zenohc::Session& z_session);

  void sampleCallback(const zenohc::Sample& sample);

  ros::NodeHandle private_nh_;

  zenohc::Session z_session_;
  zenohc::Subscriber image_sub_;
};

ImageToContour::ImageToContour(zenohc::Session& z_session) :
    private_nh_("~"),
    z_session_(std::move(z_session)),
    image_sub_(zenohc::expect<zenohc::Subscriber>(z_session_.declare_subscriber("image",
        [this](const zenohc::Sample& sample) {this->sampleCallback(sample);})))
{
  ROS_DEBUG_STREAM("moved zenoh session: " << z_session_.info_zid() << " " << &z_session_);
}

void ImageToContour::sampleCallback(const zenohc::Sample& sample)
{
  const auto z_bytes = sample.get_payload();
  const auto length = ros::serialization::serializationLength(z_bytes.get_len());
  // ROS_INFO_STREAM("sample recieved on " << sample.get_keyexpr().as_string_view() << " " << length);

  sensor_msgs::Image image_msg;
  uint8_t* bytes = const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(z_bytes.as_string_view().data()));
  ros::serialization::IStream stream(bytes, z_bytes.as_string_view().length());

  // ros::serialization::deserialize(stream, image_msg);
  ros::serialization::Serializer<sensor_msgs::Image>::read(stream, image_msg);

  ROS_INFO_STREAM(image_msg.width << "x" << image_msg.height << " " << image_msg.encoding);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "image_to_contour");
  z_owned_config_t config = z_config_default();
  ROS_INFO_STREAM("opening zeno session");
  auto z_session = zenohc::expect<zenohc::Session>(zenohc::open(std::move(config)));

  ROS_INFO_STREAM("opened zenoh session: " << z_session.info_zid() << " " << &z_session);

  ImageToContour image_to_contour(z_session);
  ros::spin();
}
