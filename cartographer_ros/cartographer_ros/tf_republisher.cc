#include "cartographer_ros/node_constants.h"

#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sstream>
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(tf_prefix, "robot_", "Prefix used by remote tf frame");
DEFINE_int32(robot_count, 0, "Total robots used");

namespace cartographer_ros
{

class TfRepublisherNode
{
  public:
    TfRepublisherNode(const std::string &tf_prefix_, const int &robot_count_);
    virtual ~TfRepublisherNode() {}

  private:
    ::ros::NodeHandle node_handle_;
    std::vector<::ros::Subscriber> pose_subscriber_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
};

TfRepublisherNode::TfRepublisherNode(const std::string &tf_prefix_, const int &robot_count_)
{
    for (int i = 0; i < robot_count_; ++i)
    {
        std::ostringstream topic;
        topic << "/" << tf_prefix_ << i << "/tf";
        ROS_INFO("Subscribing to: %s", topic.str().c_str());
        pose_subscriber_.push_back(
            node_handle_.subscribe<tf2_msgs::TFMessage>(topic.str(), kInfiniteSubscriberQueueSize,
                                                        boost::function<void(const tf2_msgs::TFMessage::ConstPtr &)>(
                                                            [this, i](const tf2_msgs::TFMessage::ConstPtr &msg) {
                                                                tf2::Transform odom_to_map;
                                                                tf2::Transform robot_to_odom;
                                                                for (const auto &x : msg->transforms)
                                                                {
                                                                    if (x.child_frame_id == "base_link" && x.header.frame_id == "odom")
                                                                    {
                                                                        tf2::fromMsg(x.transform, robot_to_odom);
                                                                    }
                                                                    else if (x.child_frame_id == "odom" && x.header.frame_id == "map")
                                                                    {
                                                                        tf2::fromMsg(x.transform, odom_to_map);
                                                                    }
                                                                }
                                                                geometry_msgs::TransformStamped stamped_transform;
                                                                stamped_transform.header.stamp = ros::Time::now();
                                                                stamped_transform.header.frame_id = "map";
                                                                stamped_transform.child_frame_id = std::string("robot_") + std::to_string(i);
                                                                stamped_transform.transform = tf2::toMsg(odom_to_map * robot_to_odom);
                                                                this->tf_broadcaster_.sendTransform(stamped_transform);
                                                            })));
    }
}

namespace
{

void Run()
{
    TfRepublisherNode node(FLAGS_tf_prefix, FLAGS_robot_count);
    ::ros::spin();
}

} // namespace
} // namespace cartographer_ros

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    CHECK(FLAGS_robot_count > 0)
        << "-robot count is missing or invalid.";

    ::ros::init(argc, argv, "tf_republish_node");
    ::ros::start();

    cartographer_ros::Run();
    ::ros::shutdown();
}