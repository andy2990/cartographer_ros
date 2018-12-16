#include <chrono>
#include <future>
#include <sstream>
#include <string>
#include <set>

#include "pcl_node.h"
#include "cartographer_ros_msgs/SubmapCloudQuery.h"
#include "cartographer_ros_msgs/StatusCode.h"
#include "cartographer_ros_msgs/StatusResponse.h"
#include <pcl/point_types.h>
#include "pcl_conversions/pcl_conversions.h"

using ::cartographer::mapping::SubmapId;

namespace cartographer_ros
{
namespace internal_impl
{

std::unique_ptr<::cartographer_ros_msgs::SubmapCloudQuery::Response> FetchSubmapCloud(
    const ::cartographer::mapping::SubmapId &submap_id,
    ros::ServiceClient *client, bool high_res, double min_probability)
{
    ::cartographer_ros_msgs::SubmapCloudQuery srv;
    srv.request.trajectory_id = submap_id.trajectory_id;
    srv.request.submap_index = submap_id.submap_index;
    srv.request.min_probability = min_probability;
    srv.request.high_resolution = high_res;
    if (!client->call(srv) ||
        srv.response.status.code != ::cartographer_ros_msgs::StatusCode::OK)
    {
        return nullptr;
    }
    auto response =
        ::cartographer::common::make_unique<::cartographer_ros_msgs::SubmapCloudQuery::Response>(srv.response);
    return response;
}
} // namespace internal_impl

PclNode::PclNode(bool high_res, float min_probability, double publish_period_sec)
    : high_res_(high_res),
      min_probability_(min_probability),
      client_(node_handle_.serviceClient<::cartographer_ros_msgs::SubmapCloudQuery>(kSubmapCloudQueryServiceName)),
      submap_list_subscriber_(node_handle_.subscribe(
          kSubmapListTopic, kLatestOnlyPublisherQueueSize,
          boost::function<void(
              const cartographer_ros_msgs::SubmapList::ConstPtr &)>(
              [this](const cartographer_ros_msgs::SubmapList::ConstPtr &msg) {
                  HandleSubmapList(msg);
              }))),
      point_cloud_publisher_(node_handle_.advertise<::sensor_msgs::PointCloud2>(
          kSubmapPointCloudTopic, kLatestOnlyPublisherQueueSize,
          true)),
      point_cloud_publisher_timer_(node_handle_.createWallTimer(::ros::WallDuration(publish_period_sec), &PclNode::PublishPointCloud, this))
{
}

void PclNode::HandleSubmapList(
    const cartographer_ros_msgs::SubmapList::ConstPtr &msg)
{
    ::cartographer::common::MutexLocker locker(&mutex_);

    // We do not do any work if nobody listens.
    if (point_cloud_publisher_.getNumSubscribers() == 0)
    {
        return;
    }

    // Keep track of submap IDs that don't appear in the message anymore.
    std::set<SubmapId> submap_ids_to_delete;
    for (const auto &pair : submap_pcls_)
    {
        submap_ids_to_delete.insert(pair.first);
    }

    for (const auto &submap_msg : msg->submap)
    {
        const SubmapId id{submap_msg.trajectory_id, submap_msg.submap_index};
        submap_ids_to_delete.erase(id);
        // fetch pointcloud
        auto &orig_pcl = submap_pcls_[id];
        auto fetched_pcl = internal_impl::FetchSubmapCloud(id, &client_, high_res_, min_probability_);
        if (fetched_pcl != nullptr)
        {
            // only update when necessary
            orig_pcl.version = fetched_pcl->submap_version;
            orig_pcl.pcl = fetched_pcl->cloud;
        }
    }

    // Delete all submaps that didn't appear in the message.
    for (const auto &id : submap_ids_to_delete)
    {
        submap_pcls_.erase(id);
    }

    last_timestamp_ = msg->header.stamp;
    last_frame_id_ = msg->header.frame_id;
}

void PclNode::PublishPointCloud(const ::ros::WallTimerEvent &unused_timer_event)
{
    ::cartographer::common::MutexLocker locker(&mutex_);

    if (point_cloud_publisher_.getNumSubscribers() == 0)
    {
        return;
    }

    sensor_msgs::PointCloud2 output;

    for (const auto &entry : submap_pcls_)
    {
        pcl::concatenatePointCloud(output, entry.second.pcl, output);
    }

    output.header.stamp = ::ros::Time::now();

    point_cloud_publisher_.publish(output);
}

} // namespace cartographer_ros