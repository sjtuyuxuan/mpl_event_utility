#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sist_event_utility/EventArray.h>
#include <iostream>
#include <string.h>

#define foreach BOOST_FOREACH

int main (int argc, char* argv[]) {
  std::string path_bag, path_out, bag_name;
  if (argc < 2) {
    ROS_ERROR("Please give the bag path");
    return 1;
  } else if (argc < 3) {
    path_bag = argv[1];
    if (path_bag.find_last_of('/') == std::string::npos) {
      path_out = "./";
      bag_name = path_bag;
    } else {
      path_out = path_bag.substr(0, path_bag.find_last_of('/') + 1);
      bag_name = path_bag.substr(path_bag.find_last_of('/') + 1,
          path_bag.find_last_of('.') - path_bag.find_last_of('/') - 1);
    }
    ROS_WARN("Output path is not set, so defult is same as bag provided");
    ROS_WARN("Output path is: \n%s", path_out.data());
  } else {
    path_bag = argv[1];
    path_out = argv[2];
    bag_name = path_bag.substr(path_bag.find_last_of('/') + 1,
        path_bag.find_last_of('.') - path_bag.find_last_of('/') - 1);
    if (path_out[path_out.size() - 1] != '/') path_out += '/';
    ROS_INFO("Output path is: \n%s", path_out.data());
  }
  rosbag::Bag bag_in;
  rosbag::Bag bag_event_left_out, bag_event_right_out, bag_rgb_left_out, bag_rgb_right_out,
              bag_lidar_out, bag_imu_out, bag_depth_raw_out, bag_depth_point_cloud_out,
              bag_depth_left_event_out;
  bag_in.open(path_bag, rosbag::bagmode::Read);
  bag_event_left_out.open(path_out + bag_name + "_event_left.bag", rosbag::bagmode::Write);
  bag_event_right_out.open(path_out + bag_name + "_event_right.bag", rosbag::bagmode::Write);
  bag_rgb_left_out.open(path_out + bag_name + "_rgb_left.bag", rosbag::bagmode::Write);
  bag_rgb_right_out.open(path_out + bag_name + "_rgb_right.bag", rosbag::bagmode::Write);
  bag_lidar_out.open(path_out + bag_name + "_lidar.bag", rosbag::bagmode::Write);
  bag_imu_out.open(path_out + bag_name + "_imu.bag", rosbag::bagmode::Write);
  bag_depth_raw_out.open(path_out + bag_name + "_depth_raw.bag", rosbag::bagmode::Write);
  bag_depth_point_cloud_out.open(path_out + bag_name + "_depth_point_cloud.bag", rosbag::bagmode::Write);
  bag_depth_left_event_out.open(path_out + bag_name + "_depth_left_event.bag", rosbag::bagmode::Write);
  bag_event_left_out.setCompression(rosbag::compression::LZ4);
  bag_event_right_out.setCompression(rosbag::compression::LZ4);
  bag_rgb_left_out.setCompression(rosbag::compression::LZ4);
  bag_rgb_right_out.setCompression(rosbag::compression::LZ4);
  bag_lidar_out.setCompression(rosbag::compression::LZ4);
  bag_depth_raw_out.setCompression(rosbag::compression::LZ4);
  bag_depth_point_cloud_out.setCompression(rosbag::compression::LZ4);
  bag_depth_left_event_out.setCompression(rosbag::compression::LZ4);


  std::vector<std::string> topics{"/prophesee/camera_left/cd_events_buffer_reconstructed",
      "/prophesee/camera_right/cd_events_buffer_reconstructed", "/stereo/left/image_mono",
      "/stereo/right/image_mono", "/os_cloud_node/points", "/imu/data", "/depth/image_raw", "/points2",
      "/depth_to_rgb/image_raw"};
  rosbag::View view(bag_in, rosbag::TopicQuery(topics));

  ros::Time left_time, right_time;
  left_time.fromNSec(0);
  right_time.fromNSec(0);
  // int count = 0;

  foreach(rosbag::MessageInstance const m, view) {
    if (m.getTopic() == "/prophesee/camera_left/cd_events_buffer_reconstructed") {
      auto s = m.instantiate<sist_event_utility::EventArray>();
      for (auto && event : s->events) {
        if (event.ts.toNSec() >= left_time.toNSec()) left_time = event.ts;
        else {
          event.ts = left_time;
          // ROS_WARN("!!! INVERSE !!! %d", ++count);
        }
        
      }
      bag_event_left_out.write("/prophesee/left/cd_events_buffer", m.getTime(), *s);
    } else if (m.getTopic() == "/prophesee/camera_right/cd_events_buffer_reconstructed") {
      auto s = m.instantiate<sist_event_utility::EventArray>();
      for (auto && event : s->events) {
        if (event.ts.toNSec() >= right_time.toNSec()) right_time = event.ts;
        else {
          event.ts = right_time;
          // ROS_WARN("!!! INVERSE !!! %d", ++count);
        }
      }
      bag_event_right_out.write("/prophesee/right/cd_events_buffer", m.getTime(), *s);
    } else if (m.getTopic() == "/stereo/left/image_mono") {
      bag_rgb_left_out.write("/stereo_rgb/left/image_mono", m.getTime(), m);
    } else if (m.getTopic() == "/stereo/right/image_mono") {
      bag_rgb_right_out.write("/stereo_rgb/right/image_mono", m.getTime(), m);
    } else if (m.getTopic() == "/os_cloud_node/points") {
      bag_lidar_out.write("/os_cloud_node/points", m.getTime(), m);
    } else if (m.getTopic() == "/imu/data") {
      bag_imu_out.write("/imu/data", m.getTime(), m);
    } else if (m.getTopic() == "/depth/image_raw") {
      bag_depth_raw_out.write("/depth/image_raw", m.getTime(), m);
    } else if (m.getTopic() == "/points2") {
      bag_depth_point_cloud_out.write("/kincet_cloud/points2", m.getTime(), m);
    } else if (m.getTopic() == "/depth_to_rgb/image_raw") {
      bag_depth_left_event_out.write("/depth_to_event_left/image_raw", m.getTime(), m);
    }
  }
  bag_in.close();
  bag_event_left_out.close();
  bag_event_right_out.close();
  bag_rgb_left_out.close();
  bag_rgb_right_out.close();
  bag_lidar_out.close();
  bag_imu_out.close();
  bag_depth_raw_out.close();
  bag_depth_point_cloud_out.close();
  bag_depth_left_event_out.close();
}