/**
 * Publish messages out of an mcap
 */
#include <chrono>
#include <functional>
#define MCAP_IMPLEMENTATION
#include <mcap/reader.hpp>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class PlayMcap : public rclcpp::Node
{
  public:
    PlayMcap() : Node("play_mcap")
    {
      std::string topic_type;
      rclcpp::QoS topic_qos_profile{1};

      const std::string inputFile = "test.mcap";
      std::ifstream input(inputFile, std::ios::binary);
      mcap::FileStreamReader dataSource{input};

      mcap::McapReader reader;
      auto status = reader.open(dataSource);
      if (!status.ok()) {
        std::cerr << "! " << status.message << "\n";
        return;
      }

      auto onProblem = [](const mcap::Status& problem) {
        std::cerr << "! " << problem.message << "\n";
      };

      auto messages = reader.readMessages(onProblem);

      // TODO(lucasw) create a map of publishers to topics like in ros1_play_mcap.py
      // std::shared_ptr<rclcpp::GenericPublisher> pub =
      //     node_->create_generic_publisher(topic_name, topic_type, topic_qos_profile);

      for (const auto& msgView : messages) {
        const mcap::Channel& channel = *msgView.channel;
        std::cout << "[" << channel.topic << "]\n";
        // TODO(lucasw) get pub from map
        // pub->publish(rclcpp::SerializedMessage((*msgView...)));
      }

      reader.close();

#if 0
     topic_type = topic.topic_metadata.type;
     topic_qos_profile = topic.topic_metadata.offered_qos_profiles[0];


      while (reader->has_next()) {
        auto msg = reader->read_next();
        // TODO Add sleep for delay between two msg->time_stamp
      }
#endif
    }

  private:
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlayMcap>());
  rclcpp::shutdown();
  return 0;
}
