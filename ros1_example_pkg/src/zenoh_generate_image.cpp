/*
 * Copyright 2023 Lucas Walter
 *
 * Generate an image and publish it in ros
 */

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <zenohc.hxx>
using namespace zenohc;

// #include <zenoh.hxx>

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

  void update(const ros::TimerEvent& event)
  {
    cv_bridge::CvImage cv_image;
    bouncing_ball_.update(cv_image.image);
    auto image_msg = cv_image.toImageMsg();
    image_msg->header.stamp = event.current_real;
    image_msg->encoding = "rgb8";
    // image_pub_.publish(image_msg);

    PublisherPutOptions options;
    // options.set_encoding(Z_ENCODING_PREFIX_TEXT_PLAIN);

    const auto length = ros::serialization::serializationLength(*image_msg);
    auto shmbuf = expect<z::Shmbuf>(z_manager_->alloc(length));

    // std::vector<uint8_t> buffer(length);
    // ros::serialization::OStream ostream(&buffer[0], length);
    // ros::serialization::serialize(ostream, float_msg);
    ros::serialization::OStream ostream(shmbuf.ptr(), length);
    ros::serialization::serialize(ostream, *image_msg);

    auto payload = shmbuf.into_payload();
    z_pub_.put_owned(std::move(payload), options);

    ROS_INFO_STREAM_THROTTLE(2.0, "published image of length: " << length);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Timer timer_;
  // ros::Publisher image_pub_;

  Session* z_session_;
  // get this error if z_manager_ is uncommented:
  //  error: use of deleted function ‘zenohc::ShmManager::ShmManager() [inherited from zenohcxx::Owned<zc_owned_shm_manager_t>]’
  ShmManager* z_manager_;
  Publisher z_pub_;

  BouncingBall bouncing_ball_;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "generate_image");
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
