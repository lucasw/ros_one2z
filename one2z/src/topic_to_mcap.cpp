/*
 * Copyright 2023 Lucas Walter
 * Read ros1 encoded messages out of an mcap file and publish them
 */

#define MCAP_IMPLEMENTATION
#include <mcap/writer.hpp>
#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
// #include <ros_type_introspection/ros_introspection.hpp>
#include <std_msgs/Float64.h>
#include <topic_tools/shape_shifter.h>

#include <fstream>
#include <signal.h>
#include <string>
#include <vector>

mcap::Timestamp mcap_now() {
  return mcap::Timestamp(std::chrono::duration_cast<std::chrono::nanoseconds>(
                           std::chrono::system_clock::now().time_since_epoch()).count());
}

class TopicToMcap
{
public:
  TopicToMcap() : nh_("~")
  {
    std::string output_file = "output.mcap";
    nh_.getParam("file", output_file);

    ROS_INFO_STREAM("saving messages to '" << output_file << "'");

    auto options = mcap::McapWriterOptions("ros1");
    options.compression = mcap::Compression::Zstd;

    out_.open(output_file.c_str(), std::ios::binary);
    writer_.open(out_, options);

    std::string topic = "/test";
    nh_.getParam("topic", topic);
    // TODO(lucasw) turn topic into full absolute namespace topic if it isn't already
    subscribe(topic);
  }

  ~TopicToMcap()
  {
    ROS_INFO_STREAM("closing");
    writer_.close();
  }

  static bool do_shutdown;  // = false;
  static void shutdown(int sig)
  {
    do_shutdown = true;
  }

  // Replacement "shutdown" XMLRPC callback
  // https://robotics.stackexchange.com/questions/37902/
  //   what-is-the-correct-way-to-do-stuff-before-a-node-is-shutdown
  static void xmlrpcShutdown(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
  {
    int num_params = 0;
    if (params.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      num_params = params.size();
    }
    if (num_params > 1) {
      std::string reason = params[1];
      ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
      do_shutdown = 1;
    }

    result = ros::xmlrpc::responseInt(1, "", 0);
  }

private:
  // Adapted from rosbag recorder.cpp
  void subscribe(const std::string& topic) {
    ROS_INFO("Subscribing to %s", topic.c_str());

    // TODO(lucasw) shapeshifter example may be better to follow but going with what is in
    // rosbag recorder for now
    // https://wiki.ros.org/ros_type_introspection/Tutorials/GenericTopicSubscriber
    ros::SubscribeOptions ops;
    ops.topic = topic;
    ops.queue_size = 100;
    ops.md5sum = ros::message_traits::md5sum<topic_tools::ShapeShifter>();
    ops.datatype = ros::message_traits::datatype<topic_tools::ShapeShifter>();
    ops.helper = boost::make_shared<ros::SubscriptionCallbackHelperT<
        const ros::MessageEvent<topic_tools::ShapeShifter const> &> >(
            boost::bind(&TopicToMcap::callback, this, boost::placeholders::_1, topic));
    sub_ = nh_.subscribe(ops);
  }

  void callback(const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event, const std::string& topic)
  {
    // TODO(lucasw) will only shutdown if receiving new topics, need a timer update to detect shutdown otherwise
    if (do_shutdown) {
      internal_shutdown();
      return;
    }
    // https://wiki.ros.org/roscpp/Overview/MessagesSerializationAndAdaptingTypes
    // see ros_comm/tools/rosbag/src/player.cpp and recorder.cpp
    // TODO(lucasw) make sure there isn't any/much cost in getting this message
    boost::shared_ptr<topic_tools::ShapeShifter const> const &msg = msg_event.getConstMessage();

    if (!schema_initialized_) {
      ROS_INFO_STREAM("initializing '" << topic << "' '"
          << msg->getDataType() << "' '" << msg->getMessageDefinition() << "'");
      // the schema is text that needs to be understood on mcap playback/decoding side
      mcap::Schema schema(msg->getDataType(), "ros1msg", msg->getMessageDefinition());
      writer_.addSchema(schema);
      channel_id_ = schema.id;

      mcap::Channel mcap_topic(topic, "ros1", channel_id_);
      writer_.addChannel(mcap_topic);
      schema_initialized_ = true;
    }

    // TODO(lucasw) msgBuf is private inside ShapeShifter, if had read only access could avoid a copy here?
    // Probably need to alter the source of shape_shifter (but already using a fork)
    // https://github.com/ros/ros_comm/blob/noetic-devel/tools/topic_tools/include/topic_tools/shape_shifter.h#L102
    const auto length = ros::serialization::serializationLength(*msg);
    std::vector<uint8_t> buffer(length);
    ros::serialization::OStream ostream(&buffer[0], length);
    ros::serialization::Serializer<topic_tools::ShapeShifter>::write(ostream, *msg);

    mcap::Message mcap_msg;
    mcap_msg.channelId = channel_id_;
    // TODO(lucasw) take this from msg
    mcap_msg.sequence = 0;
    mcap_msg.logTime = mcap_now();
    // TODO(lucasw) is this in the incoming message?
    mcap_msg.publishTime = mcap_msg.logTime;
    mcap_msg.data = reinterpret_cast<std::byte*>(buffer.data());
    mcap_msg.dataSize = length;

    const auto res = writer_.write(mcap_msg);
    if (!res.ok()) {
      ROS_ERROR_STREAM("Failed to write message: '" << res.message << "'");
      // writer_.terminate();
      // out_.close();
      // TODO(lucasw) what does this do?
      // std::ignore = std::remove(output_file);
      // TODO(lucasw) raise exception
      return;
    }
    ROS_INFO_STREAM_THROTTLE(2.0, "writing id " << channel_id_ << ", " << length << " bytes, count " << count_);
    count_++;
  }

  void internal_shutdown()
  {
    ROS_INFO_STREAM("done, shutting down writer");
    sub_.shutdown();
    writer_.close();
    ros::shutdown();
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_;

  size_t count_ = 0;

  std::ofstream out_;
  mcap::McapWriter writer_;
  bool schema_initialized_ = false;
  uint16_t channel_id_;
};

bool TopicToMcap::do_shutdown = false;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "topic_to_mcap", ros::init_options::NoSigintHandler);

  TopicToMcap topic_to_mcap;

  // TODO(lucasw) ctrl-c results in a segfault (in every C++ ros node), need to catch
  // that and shut down the writer cleanly, but for testing doing this after getting this many messages
  signal(SIGINT, topic_to_mcap.shutdown);

  // Override XMLRPC shutdown for rosnode kill
  ros::XMLRPCManager::instance()->unbind("shutdown");
  ros::XMLRPCManager::instance()->bind("shutdown", topic_to_mcap.xmlrpcShutdown);

  ros::spin();

  return 0;
}
