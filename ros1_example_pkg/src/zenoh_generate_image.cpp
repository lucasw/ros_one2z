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

#include <ros1_example_pkg/zenoh_generate_image.hpp>


GenerateImage::GenerateImage(zenohc::Session* z_session, zenohc::ShmManager* z_manager) :
    private_nh_("~"),
    z_session_(z_session),
    z_manager_(z_manager),
    z_pub_(zenohc::expect<zenohc::Publisher>(z_session->declare_publisher("image"))),
    test0_pub_(zenohc::expect<zenohc::Publisher>(z_session->declare_publisher("test0"))),
    test1_pub_(zenohc::expect<zenohc::Publisher>(z_session->declare_publisher("test1")))
{
  // image_pub_ = nh_.advertise<sensor_msgs::Image>("image", 4);
  double period = 0.033;
  private_nh_.getParam("period", period);
  timer_ = nh_.createTimer(ros::Duration(period), &GenerateImage::update, this);
}

// TODO(lucasw) any need to change return type to avoid copies?
template <typename M> zenohc::Payload
  GenerateImage::ros_msg_to_payload(const boost::shared_ptr<M>& msg)
{
  const auto length = ros::serialization::serializationLength(*msg);
  auto shmbuf = zenohc::expect<zenohc::Shmbuf>(z_manager_->alloc(length));

  ros::serialization::OStream ostream(shmbuf.ptr(), length);
  ros::serialization::serialize(ostream, *msg);

  auto payload = shmbuf.into_payload();
  return payload;
}

void GenerateImage::update(const ros::TimerEvent& event)
{
  zenohc::PublisherPutOptions options;
  zenohc::Encoding encoding;
  // TODO(lucasw) is the encoding string sent every single message?
  // would rather set it somewhere like a rosparam for the entire channel
  // (though a competing publisher may send the wrong type),
  // subscriber can get it once and assume all following messages are same type
  encoding.set_prefix(
    zenohc::EncodingPrefix::Z_ENCODING_PREFIX_APP_OCTET_STREAM);  // .set_suffix(
    // "Image");
    // this is too long
    // ros::message_traits::Definition<sensor_msgs::Image>::value());
  ROS_INFO_STREAM_ONCE(encoding.get_suffix().as_string_view());

  options.set_encoding(encoding);

  cv_bridge::CvImage cv_image;
  bouncing_ball_.update(cv_image.image);
  auto image_msg = cv_image.toImageMsg();
  image_msg->header.stamp = event.current_real;
  image_msg->encoding = "rgb8";
  z_pub_.put_owned(std::move(ros_msg_to_payload(image_msg)), options);

  ROS_INFO_STREAM_THROTTLE(2.0, "published '" << "Image"
      << ", image bytes " << image_msg->data.size()
      << " " << image_msg->width << "x" << image_msg->height);


  // TODO(lucasw) commingling this with image_pub publish using zenoh ros_comm is crashing
  auto float_msg = std_msgs::Float64();
  float_msg.data = 0.12345689101112131415;
  test0_pub_.put_owned(std::move(ros_msg_to_payload(boost::make_shared<std_msgs::Float64>(float_msg))), options);
  float_msg.data = 333.0333;
  test1_pub_.put_owned(std::move(ros_msg_to_payload(boost::make_shared<std_msgs::Float64>(float_msg))), options);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "zenoh_generate_image");
  z_owned_config_t config = z_config_default();
  ROS_INFO_STREAM("opening zeno session");
  auto z_session = zenohc::expect<zenohc::Session>(zenohc::open(std::move(config)));

  ROS_INFO_STREAM("opened zenoh session: " << z_session.info_zid());

  std::ostringstream oss;
  oss << z_session.info_zid();
  // May have to make this larger depending on bouncing ball width and height
  auto z_manager = zenohc::expect<zenohc::ShmManager>(shm_manager_new(z_session, oss.str().c_str(),
                                                      2048 * 1024 * 3 * 4));

  GenerateImage generate_image(&z_session, &z_manager);
  ros::spin();
}
