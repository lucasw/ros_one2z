/*
 * Copyright 2023 Lucas Walter
 * Example of a taking a set ros message and writing it into an mcap
 */

#define MCAP_IMPLEMENTATION
#include <mcap/writer.hpp>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <topic_tools/shape_shifter.h>

#include <fstream>
#include <string>
#include <vector>

mcap::Timestamp mcap_now() {
  return mcap::Timestamp(std::chrono::duration_cast<std::chrono::nanoseconds>(
                           std::chrono::system_clock::now().time_since_epoch()).count());
}

class MsgToMcap
{
public:
  MsgToMcap() : nh_("~")
  {
    std::string output_file = "output.mcap";
    nh_.getParam("file", output_file);

    ROS_INFO_STREAM("saving messages to '" << output_file << "'");

    mcap::McapWriter writer;

    auto options = mcap::McapWriterOptions("ros1");
    options.compression = mcap::Compression::Zstd;

    std::ofstream out(output_file.c_str(), std::ios::binary);
    writer.open(out, options);

    // https://wiki.ros.org/roscpp/Overview/MessagesSerializationAndAdaptingTypes
    std_msgs::Float64 float_msg;
    float_msg.data = 10234.35689;
    // constexpr char StringSchema[] = "string datai";

    // TODO(lucasw) this should get easier when using a generic shapeshifter subscriber
    // and all the hard-coded values can be extracted from the incoming message
    // see ros_comm/tools/rosbag/src/player.cpp and recorder.cpp
    // boost::shared_ptr<topic_tools::ShapeShifter const> const &ssmsg = float_msg;  // .getConstMessage();

    // TODO(lucasw) get message string from message?
    mcap::Schema stdMsgsString("std_msgs/Float64", "ros1msg",
        ros::message_traits::Definition<std_msgs::Float64>::value());
        // float_msg.getMessageDefinition());
    writer.addSchema(stdMsgsString);

    mcap::Channel topic("/float", "ros1", stdMsgsString.id);
    writer.addChannel(topic);

    auto start_time = mcap_now();

    for (size_t i = 0; i < 100; ++i) {
      float_msg.data += 1;
      auto length = ros::serialization::serializationLength(float_msg);
      std::vector<uint8_t> buffer(length);
      ros::serialization::OStream ostream(&buffer[0], length);
      // ros::serialization::Serializer<topic_tools::ShapeShifter>::write(ostream, *ssmsg);
      ros::serialization::serialize(ostream, float_msg);

      mcap::Message mcap_msg;
      mcap_msg.channelId = topic.id;
      mcap_msg.sequence = 0;
      mcap_msg.publishTime = start_time + i * 2e8;
      mcap_msg.logTime = mcap_msg.publishTime;
      mcap_msg.data = reinterpret_cast<std::byte*>(buffer.data());
      mcap_msg.dataSize = length;

      const auto res = writer.write(mcap_msg);
      if (!res.ok()) {
        std::cerr << "Failed to write message: " << res.message << "\n";
        writer.terminate();
        out.close();
        // TODO(lucasw) what does this do?
        // std::ignore = std::remove(output_file);
        // TODO(lucasw) raise exception
        return;
      }
    }

    writer.close();
  }

private:
  ros::NodeHandle nh_;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "msg_to_mcap");

  MsgToMcap msg_to_mcap;

  return 0;
}
