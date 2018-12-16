#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_PCL_NODE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_PCL_NODE_H
#include <sstream>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros_msgs/SubmapCloudQuery.h"
#include "cartographer_ros/submap.h"
#include "cartographer/common/mutex.h"

namespace cartographer_ros
{

class PclNode
{

    struct VersionedPointCloud
    {
        sensor_msgs::PointCloud2 pcl;
        int version;
    };

  public:
    explicit PclNode(bool high_resolution, float min_probability, double publish_period_sec);
    PclNode(const PclNode &) = delete;
    PclNode &operator=(const PclNode &) = delete;
    ~PclNode() {}

  private:
    void HandleSubmapList(const cartographer_ros_msgs::SubmapList::ConstPtr &msg);
    void PublishPointCloud(const ::ros::WallTimerEvent &timer_event);

    ::cartographer::common::Mutex mutex_;
    const bool high_res_;
    const float min_probability_;
    ::ros::NodeHandle node_handle_;
    ::ros::ServiceClient client_ GUARDED_BY(mutex_);
    ::ros::Subscriber submap_list_subscriber_ GUARDED_BY(mutex_);
    ::ros::Publisher point_cloud_publisher_ GUARDED_BY(mutex_);
    std::map<::cartographer::mapping::SubmapId, VersionedPointCloud> submap_pcls_ GUARDED_BY(mutex_);
    ::ros::WallTimer point_cloud_publisher_timer_;
    std::string last_frame_id_;
    ::ros::Time last_timestamp_;
};

} // namespace cartographer_ros

#endif