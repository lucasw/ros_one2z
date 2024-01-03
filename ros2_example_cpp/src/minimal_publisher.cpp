/**
 * Example C++ node- currently copied from docs
 * https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html#create-a-package
 * but will later adapt into an image publisher
 */
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
// #include <std_msgs/msg/int8.hpp>

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher")
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
          500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

      // serialization experiment
      {
        rclcpp::Serialization<std_msgs::msg::String> stringSerializer;
        rclcpp::SerializedMessage serialized_msg;
        stringSerializer.serialize_message(&message, &serialized_msg);
        const auto buffer_begin = serialized_msg.get_rcl_serialized_message().buffer;
        const auto buffer_end = buffer_begin + serialized_msg.size();
        std::string serialized_msg_str(buffer_begin, buffer_end);
        std::cout << "'" << serialized_msg_str << "' " <<  serialized_msg.size() << "\n";
        std::cout << "buffer: ";
        for (size_t i = 0; i < serialized_msg.size(); ++i) {
          std::cout << "[" << int(buffer_begin[i]) << "] ";
        }
        std::cout << "\n";
        std::cout << "str:    ";
        for (size_t i = 0; i < serialized_msg_str.size(); ++i) {
          std::cout << "[" << int(serialized_msg_str[i]) << "] ";
        }
        std::cout << "\n";
      }

      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
