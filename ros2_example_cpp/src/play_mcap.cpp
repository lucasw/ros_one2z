/**
 * Publish messages out of an mcap
 */
#include <chrono>
#include <functional>
#define MCAP_IMPLEMENTATION
#include <mcap/reader.hpp>
#include <map>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rcutils/types.h>

using namespace std::chrono_literals;

class PlayMcap : public rclcpp::Node
{
  public:
    PlayMcap() : Node("play_mcap")
    {
      std::string topic_type;
      rclcpp::QoS topic_qos_profile{1};

      const std::string inputFile = "test.mcap";
      RCLCPP_INFO(this->get_logger(), "Loading: '%s'", inputFile.c_str());
      std::ifstream input(inputFile, std::ios::binary);
      mcap::FileStreamReader dataSource{input};

      mcap::McapReader reader;
      const auto status0 = reader.open(dataSource);
      if (!status0.ok()) {
        std::cerr << "! " << status0.message << "\n";
        return;
      }

      auto onProblem = [](const mcap::Status& problem) {
        std::cerr << "! " << problem.message << "\n";
      };

      // TODO(lucasw) what method is best?
      auto status1 = reader.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);
      if (!status1.ok()) {
        std::cerr << "! " << status1.message << "\n";
        return;
      }

      for (const auto& it : reader.channels()) {
        const auto channel = it.second;
        const auto schema = reader.schema(channel->schemaId);
        std::cout << channel->topic << " " << channel->messageEncoding << " "
            << schema->id << " " << schema->name << " " << schema->encoding << "\n";
        // TODO(lucasw) 'reliability' is in here but not sure about the mapping
        // 'offered_qos_profiles' can have multiple profiles
        for (const auto& m_it : channel->metadata) {
          std::cout << "  " << m_it.first << ":\n"<< m_it.second << "\n";
        }
        const auto qos_profile = rclcpp::QoS(1);
        pubs[channel->topic] = this->create_generic_publisher(channel->topic, schema->name, qos_profile);
      }

      RCLCPP_INFO(this->get_logger(), "Created publishers, now publishing messages...");

      auto messages = reader.readMessages(onProblem);

      const uint64_t max_sleep = 1000000000;
      while (rclcpp::ok()) {
        mcap::Timestamp stamp;

        for (const auto& msgView : messages) {
          const mcap::Channel& channel = *msgView.channel;
          // TODO(lucasw) try except
          auto& pub = pubs[channel.topic];

          // sleep for amount of gap between messages
          // TODO(lucasw) are all messages in publishTime order?
          const auto cur_stamp = msgView.message.publishTime;
          auto delta_stamp = 0;
          if (cur_stamp > stamp) {
            delta_stamp = cur_stamp - stamp;
          } else {
            RCLCPP_WARN(this->get_logger(), "publishTime moved backwards (need to sort messages)");
          }
          if (delta_stamp < max_sleep) {
            rclcpp::sleep_for(std::chrono::nanoseconds(delta_stamp));
          } else {
            RCLCPP_INFO(this->get_logger(), "waiting for max_sleep instead of larger time");
            rclcpp::sleep_for(std::chrono::nanoseconds(max_sleep));
          }
          stamp = cur_stamp;

          // move the mcap serialized bytes into a ros2 serialized message
          const auto msg_size = msgView.message.dataSize;
          // std::cout << stamp << "ns, delta " << delta_stamp << "ns " << channel.topic
          //     << " " << msg_size << " bytes\n";

          rclcpp::SerializedMessage serialized_msg;
          serialized_msg.reserve(msg_size);
          auto& rcl_msg = serialized_msg.get_rcl_serialized_message();
          // TODO(lucasw) is this copy avoidable?
          std::memcpy(rcl_msg.buffer,
                      msgView.message.data, msg_size);
          rcl_msg.buffer_length = msg_size;
          // RCLCPP_INFO(this->get_logger(), "sizes %d %d %d", serialized_msg.capacity(),
          //             serialized_msg.size(), msg_size);
          // RCLCPP_INFO(this->get_logger(), "sizes %d %d", rcl_msg.buffer_length, rcl_msg.buffer_capacity);

          pub->publish(serialized_msg);
        }

        RCLCPP_INFO(this->get_logger(), "looping");
        rclcpp::sleep_for(std::chrono::nanoseconds(max_sleep * 2));
      }  // loop forever

      reader.close();
    }

  private:
    // TODO(lucasw) this can't handle multiple publishers on same topic of different types
    std::map<std::string, std::shared_ptr<rclcpp::GenericPublisher>> pubs;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlayMcap>());
  // auto play_mcap = std::make_shared<PlayMcap>();
  rclcpp::shutdown();
  return 0;
}
