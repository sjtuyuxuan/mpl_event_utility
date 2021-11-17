#include <boost/foreach.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sist_event_utility/EventArray.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#define foreach BOOST_FOREACH

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "timestamp_reconstructor");
  ros::NodeHandle nh;
  std::string rawbag_path, output_path, gt_path;
  if (!(ros::param::get("/rawbag_path", rawbag_path) &&
        ros::param::get("/output_path", output_path)))
    ROS_WARN("Cannot load path");
  std::string event_left_topic, event_right_topic, rgb_left_topic,
      rgb_right_topic, imu_topic, kinect_depth_topic, kinect_rgb_topic,
      lidar_topic;
  ros::param::get("/imu_topic", imu_topic);
  bool event_left  = ros::param::get("/event_left_topic", event_left_topic);
  bool event_right = ros::param::get("/event_right_topic", event_right_topic);
  bool rgb_left    = ros::param::get("/rgb_left_topic", rgb_left_topic);
  bool rgb_right   = ros::param::get("/rgb_right_topic", rgb_right_topic);
  bool kinect_depth =
      ros::param::get("/kinect_depth_topic", kinect_depth_topic);
  bool kinect_rgb = ros::param::get("/kinect_rgb_topic", kinect_rgb_topic);
  bool lidar      = ros::param::get("/lidar_topic", lidar_topic);
  bool gt         = ros::param::get("/gt_path", gt_path);

  std::string out_event_left_topic, out_event_right_topic, out_rgb_left_topic,
      out_rgb_right_topic, out_imu_topic, out_kinect_depth_topic,
      out_kinect_rgb_topic, out_lidar_topic, out_gt_topic;
  ros::param::get("/out_event_left_topic", out_event_left_topic);
  ros::param::get("/out_event_right_topic", out_event_right_topic);
  ros::param::get("/out_rgb_left_topic", out_rgb_left_topic);
  ros::param::get("/out_rgb_right_topic", out_rgb_right_topic);
  ros::param::get("/out_imu_topic", out_imu_topic);
  ros::param::get("/out_kinect_depth_topic", out_kinect_depth_topic);
  ros::param::get("/out_kinect_rgb_topic", out_kinect_rgb_topic);
  ros::param::get("/out_lidar_topic", out_lidar_topic);
  ros::param::get("/out_gt_topic", out_gt_topic);

  std::deque<sist_event_utility::EventArray::Ptr> event_left_buffer;
  std::deque<sist_event_utility::EventArray::Ptr> event_right_buffer;
  std::deque<sensor_msgs::Image::Ptr> rgb_left_buffer;
  std::deque<sensor_msgs::Image::Ptr> rgb_right_buffer;
  std::deque<sensor_msgs::Imu::Ptr> imu_buffer;
  std::vector<sensor_msgs::Image::Ptr> kinect_buffer;
  std::deque<sensor_msgs::Image::Ptr> kinect_buffer_processed;
  std::vector<sensor_msgs::Image::Ptr> kinect_buffer_rgb;
  std::deque<sensor_msgs::Image::Ptr> kinect_buffer_rgb_processed;
  std::deque<sensor_msgs::PointCloud2::Ptr> lidar_buffer;
  std::deque<geometry_msgs::PoseStamped::Ptr> gt_buffer;

  rosbag::Bag bag_in, bag_out;
  bag_in.open(rawbag_path, rosbag::bagmode::Read);
  bag_out.open(output_path, rosbag::bagmode::Write);
  bag_out.setCompression(rosbag::compression::Uncompressed);
  rosbag::View view(bag_in);
  std::ifstream gt_file;
  if (gt) {
    gt_file.open(gt_path);
    if (!gt_file.is_open())
      ROS_WARN("GT Load Unsuccess");
    else
      ROS_INFO("Start load GT from csv ...");
    std::string line;
    u_int64_t index = 0;
    ros::Time gt_time(0);
    for (int i = 0; i < 7; ++i)
      std::getline(gt_file, line);
    while (std::getline(gt_file, line)) {
      std::vector<std::string> data(9, "");
      std::istringstream readstr(line);
      for (int i = 0; i < 9; ++i) {
        std::getline(readstr, data[i], ',');
      }
      gt_buffer.emplace_back(boost::make_shared<geometry_msgs::PoseStamped>());
      gt_buffer.back()->header.seq   = index;
      gt_buffer.back()->header.stamp = ros::Time().fromNSec(index * 8333333);
      gt_buffer.back()->pose.orientation.x = atof(data[2].c_str());
      gt_buffer.back()->pose.orientation.y = atof(data[3].c_str());
      gt_buffer.back()->pose.orientation.z = atof(data[4].c_str());
      gt_buffer.back()->pose.orientation.w = atof(data[5].c_str());
      gt_buffer.back()->pose.position.x    = atof(data[6].c_str());
      gt_buffer.back()->pose.position.x    = atof(data[7].c_str());
      gt_buffer.back()->pose.position.x    = atof(data[8].c_str());
      ++index;
    }
    ROS_INFO_STREAM("Lode GT success with frame " << index);
  }

  ros::Time rgb_right_last, rgb_left_last, lidar_last, lidar_start;
  rgb_right_last.fromNSec(0);
  rgb_left_last.fromNSec(0);
  lidar_last.fromNSec(0);
  sensor_msgs::Image::Ptr rgb_right_temp, rgb_left_temp;
  sensor_msgs::PointCloud2::Ptr lidar_temp;

  // read from bag
  ROS_INFO("Start load bag ...");
  foreach (rosbag::MessageInstance const m, view) {
    if (event_left && m.getTopic() == event_left_topic)
      event_left_buffer.emplace_back(
          m.instantiate<sist_event_utility::EventArray>());
    if (event_right && m.getTopic() == event_right_topic)
      event_right_buffer.emplace_back(
          m.instantiate<sist_event_utility::EventArray>());
    if (rgb_left && m.getTopic() == rgb_left_topic) {
      auto s = m.instantiate<sensor_msgs::Image>();
      if (rgb_left_buffer.empty()) {
        if (rgb_left_last.toNSec() != 0 &&
            m.getTime().toSec() - rgb_left_last.toSec() < 0.1) {
          rgb_left_buffer.emplace_back(rgb_left_temp);
          rgb_left_buffer.emplace_back(s);
        } else {
          rgb_left_last = m.getTime();
          rgb_left_temp = s;
          ROS_DEBUG("Left Frame before start");
        }
      } else
        rgb_left_buffer.emplace_back(s);
    }
    if (rgb_right && m.getTopic() == rgb_right_topic) {
      auto s = m.instantiate<sensor_msgs::Image>();
      if (rgb_right_buffer.empty()) {
        if (rgb_right_last.toNSec() != 0 &&
            m.getTime().toSec() - rgb_right_last.toSec() < 0.1) {
          rgb_right_buffer.emplace_back(rgb_right_temp);
          rgb_right_buffer.emplace_back(s);
        } else {
          rgb_right_last = m.getTime();
          rgb_right_temp = s;
          ROS_DEBUG("Right Frame before start");
        }
      } else
        rgb_right_buffer.emplace_back(s);
    }
    if (m.getTopic() == imu_topic)
      imu_buffer.emplace_back(m.instantiate<sensor_msgs::Imu>());
    if (kinect_depth && m.getTopic() == kinect_depth_topic)
      kinect_buffer.emplace_back(m.instantiate<sensor_msgs::Image>());
    if (kinect_rgb && m.getTopic() == kinect_rgb_topic)
      kinect_buffer_rgb.emplace_back(m.instantiate<sensor_msgs::Image>());
    if (lidar && m.getTopic() == lidar_topic) {
      auto s = m.instantiate<sensor_msgs::PointCloud2>();
      if (lidar_buffer.empty()) {
        if (lidar_last.toNSec() != 0 &&
            s->header.stamp.toSec() - lidar_last.toSec() < 0.2) {
          lidar_buffer.emplace_back(lidar_temp);
          lidar_buffer.emplace_back(s);
          lidar_start = lidar_temp->header.stamp;
        } else {
          lidar_last = s->header.stamp;
          lidar_temp = s;
        }
      } else
        lidar_buffer.emplace_back(s);
    }
  }
  ROS_INFO("Load Success");

  ROS_INFO("Process Kinect ...");
  // process Kinect depth lost frame
  if (kinect_depth) {
    size_t lose_frame = 0;
    ros::Time temp(0);
    temp                           = kinect_buffer[0]->header.stamp;
    kinect_buffer[0]->header.stamp = ros::Time(0);
    kinect_buffer[0]->header.seq   = 0;
    kinect_buffer_processed.emplace_back(kinect_buffer[0]);
    for (int i = 1; i < kinect_buffer.size(); ++i) {
      if (kinect_buffer[i]->header.stamp.toSec() - temp.toSec() > 0.053) {
        ++lose_frame;
        ROS_WARN("Depth Lost One or more frame");
        if (kinect_buffer[i]->header.stamp.toSec() - temp.toSec() > 0.09) {
          ROS_WARN("Depth Lost Two or more frame");
          ++lose_frame;
        }
        if (kinect_buffer[i]->header.stamp.toSec() - temp.toSec() > 0.12) {
          ROS_ERROR("Depth Lost Three more frame");
          return -1;
        }
        if (i != kinect_buffer.size() - 1) {
          if (kinect_buffer[i + 1]->header.stamp.toSec() -
                  kinect_buffer[i]->header.stamp.toSec() <
              0.015) {
            --lose_frame;
            ROS_WARN("Depth Not Lost");
          }
        }
      }
      temp                         = kinect_buffer[i]->header.stamp;
      kinect_buffer[i]->header.seq = kinect_buffer_processed.size();
      kinect_buffer[i]->header.stamp.fromNSec(
          (kinect_buffer_processed.size() + lose_frame) * 33333333);
      kinect_buffer_processed.emplace_back(kinect_buffer[i]);
    }
    kinect_buffer.clear();
  }

  // process Kinect RGB lost frame
  if (kinect_rgb) {
    size_t lose_frame = 0;
    ros::Time temp(0);
    temp                               = kinect_buffer_rgb[0]->header.stamp;
    kinect_buffer_rgb[0]->header.stamp = ros::Time(0);
    kinect_buffer_rgb[0]->header.seq   = 0;
    kinect_buffer_rgb_processed.emplace_back(kinect_buffer_rgb[0]);
    for (int i = 1; i < kinect_buffer_rgb.size(); ++i) {
      if (kinect_buffer_rgb[i]->header.stamp.toSec() - temp.toSec() > 0.053) {
        ++lose_frame;
        ROS_WARN("RGB Lost One or more frame");
        if (kinect_buffer_rgb[i]->header.stamp.toSec() - temp.toSec() > 0.09) {
          ROS_WARN("RGB Lost Two or more frame");
          ++lose_frame;
        }
        if (kinect_buffer_rgb[i]->header.stamp.toSec() - temp.toSec() > 0.12) {
          ROS_ERROR("RGB Lost Three more frame");
          return -1;
        }
        if (i != kinect_buffer_rgb.size() - 1) {
          if (kinect_buffer_rgb[i + 1]->header.stamp.toSec() -
                  kinect_buffer_rgb[i]->header.stamp.toSec() <
              0.015) {
            --lose_frame;
            ROS_WARN("RGB Not Lost");
          }
        }
      }
      temp                             = kinect_buffer_rgb[i]->header.stamp;
      kinect_buffer_rgb[i]->header.seq = kinect_buffer_rgb_processed.size();
      kinect_buffer_rgb[i]->header.stamp.fromNSec(
          (kinect_buffer_rgb_processed.size() + lose_frame) * 33333333);
      kinect_buffer_rgb_processed.emplace_back(kinect_buffer_rgb[i]);
    }
    kinect_buffer_rgb.clear();
  }
  ROS_INFO("Process Kinect Success");

  // out put
  ros::Time start_time = imu_buffer.front()->header.stamp;

  // avoid inverse
  ros::Time left_time, right_time;
  left_time.fromNSec(0);
  right_time.fromNSec(0);
  int count = 0;

  std::vector<u_int64_t> time_now;
  time_now.emplace_back(event_left ?
                            event_left_buffer.front()->header.stamp.toNSec() :
                            UINT64_MAX);
  time_now.emplace_back(event_right ?
                            event_right_buffer.front()->header.stamp.toNSec() :
                            UINT64_MAX);
  time_now.emplace_back(rgb_left ? 0 : UINT64_MAX);
  time_now.emplace_back(rgb_right ? 0 : UINT64_MAX);
  time_now.emplace_back(0);
  time_now.emplace_back(kinect_depth ? 0 : UINT64_MAX);
  time_now.emplace_back(lidar ? 0 : UINT64_MAX);
  time_now.emplace_back(kinect_rgb ? 0 : UINT64_MAX);
  time_now.emplace_back(gt ? 0 : UINT64_MAX);

  while (!imu_buffer.empty()) {
    auto early = std::min_element(time_now.begin(), time_now.end());
    switch (static_cast<int>(std::distance(time_now.begin(), early))) {
      case 0:
        for (auto&& event : event_left_buffer.front()->events) {
          if (event.ts.toNSec() >= left_time.toNSec()) {
            left_time = event.ts;
          } else {
            event.ts = left_time;
            ROS_DEBUG("!!! INVERSE !!! %d", ++count);
          }
          event.ts.fromNSec(event.ts.toNSec() + start_time.toNSec());
        }
        event_left_buffer.front()->header.stamp.fromNSec(
            event_left_buffer.front()->header.stamp.toNSec() +
            start_time.toNSec());
        bag_out.write(out_event_left_topic,
                      event_left_buffer.front()->header.stamp,
                      event_left_buffer.front());
        event_left_buffer.pop_front();
        if (!event_left_buffer.empty()) {
          time_now[0] = event_left_buffer.front()->header.stamp.toNSec();
        } else {
          time_now[0] = UINT64_MAX;
        }
        break;
      case 1:
        for (auto&& event : event_right_buffer.front()->events) {
          if (event.ts.toNSec() >= right_time.toNSec()) {
            right_time = event.ts;
          } else {
            event.ts = right_time;
            ROS_DEBUG("!!! INVERSE !!! %d", ++count);
          }
          event.ts.fromNSec(event.ts.toNSec() + start_time.toNSec());
        }
        event_right_buffer.front()->header.stamp.fromNSec(
            event_right_buffer.front()->header.stamp.toNSec() +
            start_time.toNSec());
        bag_out.write(out_event_right_topic,
                      event_right_buffer.front()->header.stamp,
                      event_right_buffer.front());
        event_right_buffer.pop_front();
        if (!event_right_buffer.empty()) {
          time_now[1] = event_right_buffer.front()->header.stamp.toNSec();
        } else {
          time_now[1] = UINT64_MAX;
        }
        break;
      case 2:
        if (!rgb_left_buffer.empty()) {
          rgb_left_buffer.front()->header.stamp.fromNSec(time_now[2] +
                                                         start_time.toNSec());
          bag_out.write(out_rgb_left_topic,
                        rgb_left_buffer.front()->header.stamp,
                        rgb_left_buffer.front());
          time_now[2] += 33333333;
          rgb_left_buffer.pop_front();
        } else {
          time_now[2] = UINT64_MAX;
        }
        break;
      case 3:
        if (!rgb_right_buffer.empty()) {
          rgb_right_buffer.front()->header.stamp.fromNSec(time_now[3] +
                                                          start_time.toNSec());
          bag_out.write(out_rgb_right_topic,
                        rgb_right_buffer.front()->header.stamp,
                        rgb_right_buffer.front());
          time_now[3] += 33333333;
          rgb_right_buffer.pop_front();
        } else {
          time_now[3] = UINT64_MAX;
        }
        break;
      case 4:
        if (!imu_buffer.empty()) {
          imu_buffer.front()->header.stamp.fromNSec(time_now[4] +
                                                    start_time.toNSec());
          bag_out.write(out_imu_topic,
                        imu_buffer.front()->header.stamp,
                        imu_buffer.front());
          time_now[4] += 5000000;
          imu_buffer.pop_front();
        } else {
          time_now[4] = UINT64_MAX;
        }
        break;
      case 5:
        if (!kinect_buffer_processed.empty()) {
          time_now[5] =
              kinect_buffer_processed.front()->header.stamp.toNSec() + 33333333;
          kinect_buffer_processed.front()->header.stamp.fromNSec(
              kinect_buffer_processed.front()->header.stamp.toNSec() +
              start_time.toNSec());
          bag_out.write(out_kinect_depth_topic,
                        kinect_buffer_processed.front()->header.stamp,
                        kinect_buffer_processed.front());
          kinect_buffer_processed.pop_front();
        } else {
          time_now[5] = UINT64_MAX;
        }
        break;
      case 6:
        if (!lidar_buffer.empty()) {
          lidar_buffer.front()->header.stamp.fromNSec(
              lidar_buffer.front()->header.stamp.toNSec() -
              lidar_start.toNSec() + start_time.toNSec());
          bag_out.write(out_lidar_topic,
                        lidar_buffer.front()->header.stamp,
                        lidar_buffer.front());
          time_now[6] = lidar_buffer.front()->header.stamp.toNSec() +
                        100000000 - start_time.toNSec();
          lidar_buffer.pop_front();
        } else {
          time_now[6] = UINT64_MAX;
        }
        break;
      case 7:
        if (!kinect_buffer_rgb_processed.empty()) {
          time_now[7] =
              kinect_buffer_rgb_processed.front()->header.stamp.toNSec() +
              33333333;
          kinect_buffer_rgb_processed.front()->header.stamp.fromNSec(
              kinect_buffer_rgb_processed.front()->header.stamp.toNSec() +
              start_time.toNSec());
          bag_out.write(out_kinect_rgb_topic,
                        kinect_buffer_rgb_processed.front()->header.stamp,
                        kinect_buffer_rgb_processed.front());
          kinect_buffer_rgb_processed.pop_front();
        } else {
          time_now[7] = UINT64_MAX;
        }
        break;
      case 8:
        if (!gt_buffer.empty()) {
          time_now[8] = gt_buffer.front()->header.stamp.toNSec() + 8333333;
          gt_buffer.front()->header.stamp.fromNSec(
              gt_buffer.front()->header.stamp.toNSec() + start_time.toNSec());
          bag_out.write(
              out_gt_topic, gt_buffer.front()->header.stamp, gt_buffer.front());
          gt_buffer.pop_front();
        } else {
          time_now[8] = UINT64_MAX;
        }
        break;
    }
  }
}