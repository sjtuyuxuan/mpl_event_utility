#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/ros.h>
#include <boost/foreach.hpp>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sist_event_utility/EventArray.h>

#include <iostream>
#include <algorithm>
#include <string.h>


#define foreach BOOST_FOREACH

int main (int argc, char* argv[]) {
  ros::init(argc, argv, "timestamp_reconstructor");
  ros::NodeHandle nh;
  std::string rawbag_path, output_path;
  if (!(ros::param::get("/rawbag_path", rawbag_path) && ros::param::get("/output_path", output_path)))
      ROS_WARN("Cannot load path");
  std::string event_left_topic, event_right_topic, rgb_left_topic, rgb_right_topic, imu_topic, kinect_topic, lidar_topic;
  ros::param::get("/event_left_topic", event_left_topic);
  ros::param::get("/event_right_topic", event_right_topic);
  ros::param::get("/rgb_left_topic", rgb_left_topic);
  ros::param::get("/rgb_right_topic", rgb_right_topic);
  ros::param::get("/imu_topic", imu_topic);
  bool hand_held = ros::param::get("/kinect_topic", kinect_topic);
  bool wide = ros::param::get("/lidar_topic", lidar_topic);

  std::deque<sist_event_utility::EventArray::Ptr> event_left_buffer;
  std::deque<sist_event_utility::EventArray::Ptr> event_right_buffer;
  std::deque<sensor_msgs::Image::Ptr> rgb_left_buffer;
  std::deque<sensor_msgs::Image::Ptr> rgb_right_buffer;
  std::deque<sensor_msgs::Imu::Ptr> imu_buffer;
  std::vector<sensor_msgs::Image::Ptr> kinect_buffer;
  std::deque<sensor_msgs::Image::Ptr> kinect_buffer_processed;
  std::deque<sensor_msgs::PointCloud2::Ptr> lidar_buffer;
  
  rosbag::Bag bag_in, bag_out;
  bag_in.open(rawbag_path, rosbag::bagmode::Read);
  bag_out.open(output_path, rosbag::bagmode::Write);
  bag_out.setCompression(rosbag::compression::Uncompressed);
  rosbag::View view(bag_in);


  ros::Time rgb_right_last, rgb_left_last, lidar_last, lidar_start;
  rgb_right_last.fromNSec(0);
  rgb_left_last.fromNSec(0);
  lidar_last.fromNSec(0);


  // read from bag
  foreach(rosbag::MessageInstance const m, view) {
    if (m.getTopic() == event_left_topic) event_left_buffer.emplace_back(m.instantiate<sist_event_utility::EventArray>());
    if (m.getTopic() == event_right_topic) event_right_buffer.emplace_back(m.instantiate<sist_event_utility::EventArray>());
    if (m.getTopic() == rgb_left_topic) {
      auto s = m.instantiate<sensor_msgs::Image>();
      if (rgb_left_buffer.empty()) {
        if (rgb_left_last.toNSec() != 0 && m.getTime().toSec() - rgb_right_last.toSec() < 0.1) rgb_left_buffer.emplace_back(s);
        else rgb_left_last = m.getTime();
      } else rgb_left_buffer.emplace_back(s);
    }
    if (m.getTopic() == rgb_right_topic) {
      auto s = m.instantiate<sensor_msgs::Image>();
      if (rgb_right_buffer.empty()) {
        if (rgb_right_last.toNSec() != 0 && m.getTime().toSec() - rgb_right_last.toSec() < 0.1) rgb_right_buffer.emplace_back(s);
        else rgb_right_last = m.getTime();
      } else rgb_right_buffer.emplace_back(s);
    }
    if (m.getTopic() == imu_topic) imu_buffer.emplace_back(m.instantiate<sensor_msgs::Imu>());
    if (hand_held)
        if (m.getTopic() == kinect_topic) kinect_buffer.emplace_back(m.instantiate<sensor_msgs::Image>());
    if (wide) {
      if (m.getTopic() == lidar_topic) {
        auto s = m.instantiate<sensor_msgs::PointCloud2>();
        if (lidar_buffer.empty()) {
          if (lidar_last.toNSec() != 0 && s->header.stamp.toSec() - lidar_last.toSec() < 0.2) {
            lidar_buffer.emplace_back(s);
            lidar_start = s->header.stamp;
          } else lidar_last = s->header.stamp;
        } else lidar_buffer.emplace_back(s);
      }
    }
  }
  ROS_INFO("Load Success");

  // process Kinect lost frame
  if (hand_held){
    size_t lose_frame = 0;
    ros::Time temp(0);
    temp = kinect_buffer[0]->header.stamp;
    kinect_buffer[0]->header.stamp = ros::Time(0);
    kinect_buffer[0]->header.seq = 0;
    kinect_buffer_processed.emplace_back(kinect_buffer[0]);
    for (int i = 1 ; i < kinect_buffer.size(); ++i) {
      if (kinect_buffer[i]->header.stamp.toSec() - temp.toSec() > 0.053) {
        ++lose_frame;
        ROS_WARN("Lost One or more frame");
        if (kinect_buffer[i]->header.stamp.toSec() - temp.toSec() > 0.09) {
          ROS_WARN("Lost Two or more frame");
          ++lose_frame;
        }
        if (kinect_buffer[i]->header.stamp.toSec() - temp.toSec() > 0.12) {
          ROS_ERROR("Lost Three more frame");
          return -1;
        }
        if (i != kinect_buffer.size() - 1) {
          if (kinect_buffer[i+1]->header.stamp.toSec() - kinect_buffer[i]->header.stamp.toSec() < 0.015) {
            --lose_frame;
            ROS_WARN("Not Lost");
          }
        }
      }
      temp = kinect_buffer[i]->header.stamp;
      kinect_buffer[i]->header.seq = kinect_buffer_processed.size();
      kinect_buffer[i]->header.stamp.fromNSec((kinect_buffer_processed.size() + lose_frame) * 33333333);
      kinect_buffer_processed.emplace_back(kinect_buffer[i]);
    }
    kinect_buffer.clear();
  }
  ROS_INFO("Process Kinect Success");

  // out put
  ros::Time start_time = imu_buffer.front()->header.stamp;
  
  // avoid inverse
  ros::Time left_time, right_time;
  left_time.fromNSec(0);
  right_time.fromNSec(0);
  int count = 0;

  std::vector<u_int64_t> time_now = {event_left_buffer.front()->header.stamp.toNSec(),
      event_right_buffer.front()->header.stamp.toNSec(), 0, 0, 0};
  if (hand_held) time_now.emplace_back(0);
  else time_now.emplace_back(UINT64_MAX);
  if (wide) time_now.emplace_back(0);
  else time_now.emplace_back(UINT64_MAX);
  while (!imu_buffer.empty())
  {
    auto early = std::min_element(time_now.begin(), time_now.end());
    switch (static_cast<int>(std::distance(time_now.begin(), early))) {
      case 0:
        for (auto && event : event_left_buffer.front()->events) {
          if (event.ts.toNSec() >= left_time.toNSec()) {
            left_time = event.ts;
          } else {
            event.ts = left_time;
            ROS_DEBUG("!!! INVERSE !!! %d", ++count);
          }
          event.ts.fromNSec(event.ts.toNSec() + start_time.toNSec());
        }
        event_left_buffer.front()->header.stamp.fromNSec(
            event_left_buffer.front()->header.stamp.toNSec() + start_time.toNSec());
        bag_out.write("/prophesee/left/cd_events_buffer",
            event_left_buffer.front()->header.stamp, event_left_buffer.front());
        event_left_buffer.pop_front();
        if (!event_left_buffer.empty()) {
          time_now[0] = event_left_buffer.front()->header.stamp.toNSec();
        } else {
          time_now[0] = UINT64_MAX;
        }
        break;
      case 1:
        for (auto && event : event_right_buffer.front()->events) {
          if (event.ts.toNSec() >= right_time.toNSec()) {
            right_time = event.ts;
          } else {
            event.ts = right_time;
            ROS_DEBUG("!!! INVERSE !!! %d", ++count);
          }
          event.ts.fromNSec(event.ts.toNSec() + start_time.toNSec());
        }
        event_right_buffer.front()->header.stamp.fromNSec(
            event_right_buffer.front()->header.stamp.toNSec() + start_time.toNSec());
        bag_out.write("/prophesee/right/cd_events_buffer",
            event_right_buffer.front()->header.stamp, event_right_buffer.front());
        event_right_buffer.pop_front();
        if (!event_right_buffer.empty()) {
          time_now[1] = event_right_buffer.front()->header.stamp.toNSec();
        } else {
          time_now[1] = UINT64_MAX;
        }
        break;
      case 2:
        if (!rgb_left_buffer.empty()) {
          rgb_left_buffer.front()->header.stamp.fromNSec(time_now[2] + start_time.toNSec());
          bag_out.write("/stereo/left/image_mono",
              rgb_left_buffer.front()->header.stamp, rgb_left_buffer.front());
          time_now[2] += 33333333;
          rgb_left_buffer.pop_front();
        } else {
          time_now[2] = UINT64_MAX;
        }
        break;
      case 3:
        if (!rgb_right_buffer.empty()) {
          rgb_right_buffer.front()->header.stamp.fromNSec(time_now[3] + start_time.toNSec());
          bag_out.write("/stereo/right/image_mono",
              rgb_right_buffer.front()->header.stamp, rgb_right_buffer.front());
          time_now[3] += 33333333;
          rgb_right_buffer.pop_front();
        } else {
          time_now[3] = UINT64_MAX;
        }
        break;
      case 4:
        if (!imu_buffer.empty()) {
          imu_buffer.front()->header.stamp.fromNSec(time_now[4] + start_time.toNSec());
          bag_out.write("/imu/data",
              imu_buffer.front()->header.stamp, imu_buffer.front());
          time_now[4] += 5000000;
          imu_buffer.pop_front();
        } else {
          time_now[4] = UINT64_MAX;
        }
        break;
      case 5:
        if (!kinect_buffer_processed.empty()) {
          time_now[5] = kinect_buffer_processed.front()->header.stamp.toNSec() + 33333333;
          kinect_buffer_processed.front()->header.stamp.fromNSec(
              kinect_buffer_processed.front()->header.stamp.toNSec() + start_time.toNSec());
          bag_out.write("/kinect/depth_image",
              kinect_buffer_processed.front()->header.stamp, kinect_buffer_processed.front());
          kinect_buffer_processed.pop_front();
        } else {
          time_now[5] = UINT64_MAX;
        }
        break;
      case 6:
        if (!lidar_buffer.empty()) {
          lidar_buffer.front()->header.stamp.fromNSec(
              lidar_buffer.front()->header.stamp.toNSec() - lidar_start.toNSec() + start_time.toNSec());
          bag_out.write("/os_cloud_node/points",
              lidar_buffer.front()->header.stamp, lidar_buffer.front());
          time_now[6] = lidar_buffer.front()->header.stamp.toNSec() - start_time.toNSec() + 100000000;
          lidar_buffer.pop_front();
        } else {
          time_now[6] = UINT64_MAX;
        }
        break;
    }
  }
}