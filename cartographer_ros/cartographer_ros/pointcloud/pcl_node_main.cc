#include "pcl_node.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_bool(high_resolution, true,
            "Enable high resolution pointcloud.");

DEFINE_double(publish_period_sec, 1.0,
              "Pointcloud publishing period.");

DEFINE_double(min_probability, 0.6,
             "Min probability of occupation");

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    ::ros::init(argc, argv, "cartographer_point_cloud_node");
    ::ros::start();

    cartographer_ros::PclNode node(FLAGS_high_resolution, FLAGS_min_probability, FLAGS_publish_period_sec);

    ::ros::spin();
    ::ros::shutdown();
}