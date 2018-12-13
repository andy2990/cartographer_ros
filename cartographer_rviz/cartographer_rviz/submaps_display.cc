/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer_rviz/submaps_display.h"

#include "OgreResourceGroupManager.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/mutex.h"
#include "cartographer/mapping/id.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "cartographer_ros_msgs/SubmapCloudQuery.h"
#include "geometry_msgs/TransformStamped.h"
#include "pluginlib/class_list_macros.h"
#include "ros/package.h"
#include "ros/ros.h"
#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/properties/bool_property.h"
#include "rviz/properties/string_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/default_plugin/point_cloud_common.h"
#include "rviz/default_plugin/point_cloud_transformers.h"
#include "rviz/ogre_helpers/point_cloud.h"
#include "rviz/validate_floats.h"

namespace cartographer_rviz {

namespace {

constexpr int kMaxOnGoingRequestsPerTrajectory = 12;
constexpr char kMaterialsDirectory[] = "/ogre_media/materials";
constexpr char kGlsl120Directory[] = "/glsl120";
constexpr char kScriptsDirectory[] = "/scripts";
constexpr char kDefaultTrackingFrame[] = "base_link";
constexpr char kDefaultSubmapQueryServiceName[] = "/submap_query";
constexpr char kDefaultSubmapCloudQueryServiceName[] = "/submap_cloud_query";

}  // namespace

SubmapsDisplay::SubmapsDisplay() : tf_listener_(tf_buffer_), point_cloud_common_( new ::rviz::PointCloudCommon( this )) 
{
  submap_query_service_property_ = new ::rviz::StringProperty(
      "Submap query service", kDefaultSubmapQueryServiceName,
      "Submap query service to connect to.", this, SLOT(Reset()));
  submap_cloud_query_service_property_ = new ::rviz::StringProperty(
      "Submap pointcloud query service", kDefaultSubmapCloudQueryServiceName,
      "Submap pointcloud query service to connect to.", this, SLOT(ResetCloud()));
  tracking_frame_property_ = new ::rviz::StringProperty(
      "Tracking frame", kDefaultTrackingFrame,
      "Tracking frame, used for fading out submaps.", this);
  slice_high_resolution_enabled_ = new ::rviz::BoolProperty(
      "High Resolution", true, "Display high resolution slices.", this,
      SLOT(ResolutionToggled()), this);
  slice_low_resolution_enabled_ = new ::rviz::BoolProperty(
      "Low Resolution", false, "Display low resolution slices.", this,
      SLOT(ResolutionToggled()), this);
  client_ = update_nh_.serviceClient<::cartographer_ros_msgs::SubmapQuery>("");
  cloud_client_ = update_nh_.serviceClient<::cartographer_ros_msgs::SubmapCloudQuery>("");
  trajectories_category_ = new ::rviz::Property(
      "Submaps", QVariant(), "List of all submaps, organized by trajectories.",
      this);
  visibility_all_enabled_ = new ::rviz::BoolProperty(
      "All", true,
      "Whether submaps from all trajectories should be displayed or not.",
      trajectories_category_, SLOT(AllEnabledToggled()), this);
  fade_out_start_distance_in_meters_ =
      new ::rviz::FloatProperty("Fade-out distance", 1.f,
                                "Distance in meters in z-direction beyond "
                                "which submaps will start to fade out.",
                                this);
  queue_size_property_ = new ::rviz::IntProperty( "Queue Size", 10,
                                          "Advanced: set the size of the incoming PointCloud2 message queue. "
                                          " Increasing this is useful if your incoming TF data is delayed significantly "
                                          "from your PointCloud2 data, but it can greatly increase memory usage if the messages are big.",
                                          this, SLOT( updateQueueSize() ));
  const std::string package_path = ::ros::package::getPath(ROS_PACKAGE_NAME);
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      package_path + kMaterialsDirectory, "FileSystem", ROS_PACKAGE_NAME);
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      package_path + kMaterialsDirectory + kGlsl120Directory, "FileSystem",
      ROS_PACKAGE_NAME);
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      package_path + kMaterialsDirectory + kScriptsDirectory, "FileSystem",
      ROS_PACKAGE_NAME);
  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

  update_nh_.setCallbackQueue( point_cloud_common_->getCallbackQueue() );
}

SubmapsDisplay::~SubmapsDisplay() {
  client_.shutdown();
  cloud_client_.shutdown();
  trajectories_.clear();
  scene_manager_->destroySceneNode(map_node_);
  delete point_cloud_common_;
}

void SubmapsDisplay::ProcessPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  processMessage(cloud);
}

void SubmapsDisplay::updateQueueSize()
{
  tf_filter_->setQueueSize( (uint32_t) queue_size_property_->getInt() );
}

void SubmapsDisplay::Reset() { reset(); }

void SubmapsDisplay::CreateClient() {
  client_ = update_nh_.serviceClient<::cartographer_ros_msgs::SubmapQuery>(
      submap_query_service_property_->getStdString());
}

void SubmapsDisplay::CreateCloudClient() {
  cloud_client_ = update_nh_.serviceClient<::cartographer_ros_msgs::SubmapCloudQuery>(
    submap_cloud_query_service_property_->getStdString());
}

void SubmapsDisplay::onInitialize() {
  MFDClass::onInitialize();
  map_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  CreateClient();
  CreateCloudClient();
  point_cloud_common_->initialize( context_, scene_node_ );
}

void SubmapsDisplay::reset() {
  MFDClass::reset();
  ::cartographer::common::MutexLocker locker(&mutex_);
  client_.shutdown();
  trajectories_.clear();
  CreateClient();
}

void SubmapsDisplay::ResetCloud() {
  MFDClass::reset();
  ::cartographer::common::MutexLocker locker(&mutex_);
  point_cloud_common_->reset();
  cloud_client_.shutdown();
  CreateCloudClient();
}

void SubmapsDisplay::processMessage(
    const ::cartographer_ros_msgs::SubmapList::ConstPtr& msg) {
  ::cartographer::common::MutexLocker locker(&mutex_);
  map_frame_ =
      ::cartographer::common::make_unique<std::string>(msg->header.frame_id);
  // In case Cartographer node is relaunched, destroy trajectories from the
  // previous instance.
  for (const ::cartographer_ros_msgs::SubmapEntry& submap_entry : msg->submap) {
    const size_t trajectory_id = submap_entry.trajectory_id;
    if (trajectory_id >= trajectories_.size()) {
      continue;
    }
    const auto& trajectory_submaps = trajectories_[trajectory_id]->submaps;
    const auto it = trajectory_submaps.find(submap_entry.submap_index);
    if (it != trajectory_submaps.end() &&
        it->second->version() > submap_entry.submap_version) {
      // Versions should only increase unless Cartographer restarted.
      trajectories_.clear();
      break;
    }
  }
  using ::cartographer::mapping::SubmapId;
  std::set<SubmapId> listed_submaps;
  for (const ::cartographer_ros_msgs::SubmapEntry& submap_entry : msg->submap) {
    const SubmapId id{submap_entry.trajectory_id, submap_entry.submap_index};
    listed_submaps.insert(id);
    while (id.trajectory_id >= static_cast<int>(trajectories_.size())) {
      trajectories_.push_back(::cartographer::common::make_unique<Trajectory>(
          ::cartographer::common::make_unique<::rviz::BoolProperty>(
              QString("Trajectory %1").arg(id.trajectory_id),
              visibility_all_enabled_->getBool(),
              QString("List of all submaps in Trajectory %1. The checkbox "
                      "controls whether all submaps in this trajectory should "
                      "be displayed or not.")
                  .arg(id.trajectory_id),
              trajectories_category_)));
    }
    auto& trajectory_visibility = trajectories_[id.trajectory_id]->visibility;
    auto& trajectory_submaps = trajectories_[id.trajectory_id]->submaps;
    if (trajectory_submaps.count(id.submap_index) == 0) {
      // TODO(ojura): Add RViz properties for adjusting submap pose axes
      constexpr float kSubmapPoseAxesLength = 0.3f;
      constexpr float kSubmapPoseAxesRadius = 0.06f;
      trajectory_submaps.emplace(
          id.submap_index,
          ::cartographer::common::make_unique<DrawableSubmap>(
              id, context_, map_node_, trajectory_visibility.get(),
              trajectory_visibility->getBool(), kSubmapPoseAxesLength,
              kSubmapPoseAxesRadius, this));
      trajectory_submaps.at(id.submap_index)
          ->SetSliceVisibility(0, slice_high_resolution_enabled_->getBool());
      trajectory_submaps.at(id.submap_index)
          ->SetSliceVisibility(1, slice_low_resolution_enabled_->getBool());
    }
    trajectory_submaps.at(id.submap_index)->Update(msg->header, submap_entry);
  }
  // Remove all submaps not mentioned in the SubmapList.
  for (size_t trajectory_id = 0; trajectory_id < trajectories_.size();
       ++trajectory_id) {
    auto& trajectory_submaps = trajectories_[trajectory_id]->submaps;
    for (auto it = trajectory_submaps.begin();
         it != trajectory_submaps.end();) {
      if (listed_submaps.count(
              SubmapId{static_cast<int>(trajectory_id), it->first}) == 0) {
        it = trajectory_submaps.erase(it);
      } else {
        ++it;
      }
    }
  }
}

void SubmapsDisplay::processMessage( const sensor_msgs::PointCloud2ConstPtr& cloud )
{
  using namespace rviz;
  // Filter any nan values out of the cloud.  Any nan values that make it through to PointCloudBase
  // will get their points put off in lala land, but it means they still do get processed/rendered
  // which can be a big performance hit
  sensor_msgs::PointCloud2Ptr filtered(new sensor_msgs::PointCloud2);
  int32_t xi = findChannelIndex(cloud, "x");
  int32_t yi = findChannelIndex(cloud, "y");
  int32_t zi = findChannelIndex(cloud, "z");

  if (xi == -1 || yi == -1 || zi == -1)
  {
    return;
  }

  const uint32_t xoff = cloud->fields[xi].offset;
  const uint32_t yoff = cloud->fields[yi].offset;
  const uint32_t zoff = cloud->fields[zi].offset;
  const uint32_t point_step = cloud->point_step;
  const size_t point_count = cloud->width * cloud->height;

  if( point_count * point_step != cloud->data.size() )
  {
    std::stringstream ss;
    ss << "Data size (" << cloud->data.size() << " bytes) does not match width (" << cloud->width
       << ") times height (" << cloud->height << ") times point_step (" << point_step << ").  Dropping message.";
    setStatusStd( StatusProperty::Error, "Message", ss.str() );
    return;
  }

  filtered->data.resize(cloud->data.size());
  uint32_t output_count;
  if (point_count == 0)
  {
    output_count = 0;
  } else {
    uint8_t* output_ptr = &filtered->data.front();
    const uint8_t* ptr = &cloud->data.front(), *ptr_end = &cloud->data.back(), *ptr_init;
    size_t points_to_copy = 0;
    for (; ptr < ptr_end; ptr += point_step)
    {
      float x = *reinterpret_cast<const float*>(ptr + xoff);
      float y = *reinterpret_cast<const float*>(ptr + yoff);
      float z = *reinterpret_cast<const float*>(ptr + zoff);
      if (validateFloats(x) && validateFloats(y) && validateFloats(z))
      {
        if (points_to_copy == 0)
        {
          // Only memorize where to start copying from
          ptr_init = ptr;
          points_to_copy = 1;
        }
        else
        {
          ++points_to_copy;
        }
      }
      else
      {
        if (points_to_copy)
        {
          // Copy all the points that need to be copied
          memcpy(output_ptr, ptr_init, point_step*points_to_copy);
          output_ptr += point_step*points_to_copy;
          points_to_copy = 0;
        }
      }
    }
    // Don't forget to flush what needs to be copied
    if (points_to_copy)
    {
      memcpy(output_ptr, ptr_init, point_step*points_to_copy);
      output_ptr += point_step*points_to_copy;
    }
    output_count = (output_ptr - &filtered->data.front()) / point_step;
  }

  filtered->header = cloud->header;
  filtered->fields = cloud->fields;
  filtered->data.resize(output_count * point_step);
  filtered->height = 1;
  filtered->width = output_count;
  filtered->is_bigendian = cloud->is_bigendian;
  filtered->point_step = point_step;
  filtered->row_step = output_count;

  point_cloud_common_->addMessage( filtered );
}


void SubmapsDisplay::update(const float wall_dt, const float ros_dt) {
  ::cartographer::common::MutexLocker locker(&mutex_);

  point_cloud_common_->update( wall_dt, ros_dt );

  // Schedule fetching of new submap textures.
  for (const auto& trajectory : trajectories_) {
    int num_ongoing_requests = 0;
    for (const auto& submap_entry : trajectory->submaps) {
      if (submap_entry.second->QueryInProgress()) {
        ++num_ongoing_requests;
      }
    }
    for (auto it = trajectory->submaps.rbegin();
         it != trajectory->submaps.rend() &&
         num_ongoing_requests < kMaxOnGoingRequestsPerTrajectory;
         ++it) {
      if (it->second->MaybeFetchTexture(&client_)) {
        ++num_ongoing_requests;
      }
      if(it->second->MaybeFetchPointCloud(&cloud_client_))
      {
        ++num_ongoing_requests;
      }
    }
  }
  if (map_frame_ == nullptr) {
    return;
  }
  // Update the fading by z distance.
  const ros::Time kLatest(0);
  try {
    const ::geometry_msgs::TransformStamped transform_stamped =
        tf_buffer_.lookupTransform(
            *map_frame_, tracking_frame_property_->getStdString(), kLatest);
    for (auto& trajectory : trajectories_) {
      for (auto& submap_entry : trajectory->submaps) {
        submap_entry.second->SetAlpha(
            transform_stamped.transform.translation.z,
            fade_out_start_distance_in_meters_->getFloat());
      }
    }
  } catch (const tf2::TransformException& ex) {
    ROS_WARN_THROTTLE(1., "Could not compute submap fading: %s", ex.what());
  }
  // Update the map frame to fixed frame transform.
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (context_->getFrameManager()->getTransform(*map_frame_, kLatest, position,
                                                orientation)) {
    map_node_->setPosition(position);
    map_node_->setOrientation(orientation);
    context_->queueRender();
  }
}

void SubmapsDisplay::AllEnabledToggled() {
  ::cartographer::common::MutexLocker locker(&mutex_);
  const bool visible = visibility_all_enabled_->getBool();
  for (auto& trajectory : trajectories_) {
    trajectory->visibility->setBool(visible);
  }
}

void SubmapsDisplay::ResolutionToggled() {
  ::cartographer::common::MutexLocker locker(&mutex_);
  for (auto& trajectory : trajectories_) {
    for (auto& submap_entry : trajectory->submaps) {
      submap_entry.second->SetSliceVisibility(
          0, slice_high_resolution_enabled_->getBool());
      submap_entry.second->SetSliceVisibility(
          1, slice_low_resolution_enabled_->getBool());
    }
  }
}

void Trajectory::AllEnabledToggled() {
  const bool visible = visibility->getBool();
  for (auto& submap_entry : submaps) {
    submap_entry.second->set_visibility(visible);
  }
}

Trajectory::Trajectory(std::unique_ptr<::rviz::BoolProperty> property)
    : visibility(std::move(property)) {
  ::QObject::connect(visibility.get(), SIGNAL(changed()), this,
                     SLOT(AllEnabledToggled()));
}

}  // namespace cartographer_rviz

PLUGINLIB_EXPORT_CLASS(cartographer_rviz::SubmapsDisplay, ::rviz::Display)
