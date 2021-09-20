#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sist_event_utility/EventArray.h>
#include <iostream>
#include <boost/filesystem.hpp>
#include <string.h>

#define foreach BOOST_FOREACH

int get_earliest (
    std::vector<std::queue<rosbag::MessageInstance>>& queue_for_merge) {
  ros::Time time_earliest;
  time_earliest.fromSec(2000000000);
  int index;
  for (size_t i = 0; i < queue_for_merge.size(); ++i) {
    if (queue_for_merge[i].empty()) {
      continue;
    } else if (queue_for_merge[i].front().getTime().toNSec() < time_earliest.toNSec()) {
      index = i;
      time_earliest = queue_for_merge[i].front().getTime();
    }
  }
  if (time_earliest.toSec() < 1700000000) return index;
  else return -1;
}

int main (int argc, char* argv[]) {
  std::string directory_path;
  boost::filesystem::path directory;
  rosbag::Bag bag_out;
  std::vector<std::string> bags_path;
  rosbag::Bag bags[10];
  rosbag::View bags_views[10];
  std::vector<std::pair<rosbag::View::iterator,rosbag::View::iterator>> bags_views_its;
  std::vector<std::queue<rosbag::MessageInstance>> queue_for_merge;
  if (argc < 2) {
    ROS_ERROR("Please give the directory");
    return 1;
  } else {
    directory_path =  argv[1];
    directory = argv[1];
    boost::filesystem::directory_iterator end_iter;
    for(boost::filesystem::directory_iterator iter(directory); iter != end_iter; ++iter) {
      if (boost::filesystem::is_regular_file(iter->status())) {
        ROS_INFO("We get a bag in: %s\n", iter->path().string().data());
        bags_path.emplace_back(iter->path().string());
      } 
    }
    if (bags_path.size() < 2) {
      ROS_ERROR("No more than 2 bags. Please check!");
    }
  }
  for (size_t i = 0; i < bags_path.size(); ++i) {
    bags[i].open(bags_path[i], rosbag::bagmode::Read);
    bags_views[i].addQuery(bags[i]);
    bags_views_its.emplace_back(bags_views[i].begin(), bags_views[i].end());
  }
  if (directory_path.back() != '/') directory_path += '/';
  bag_out.open(directory_path + "out_merged.bag", rosbag::bagmode::Write);
  queue_for_merge.resize(bags_path.size());
  while(true) {
    for (size_t i = 0; i < bags_views_its.size(); ++i) {
      if (bags_views_its[i].first != bags_views_its[i].second &&
          queue_for_merge[i].empty()) {
        queue_for_merge[i].push(*bags_views_its[i].first);
        ++bags_views_its[i].first;
      }
    }
    int index = get_earliest(queue_for_merge);
    if (index != -1) {
      bag_out.write(queue_for_merge[index].front().getTopic(),
                    queue_for_merge[index].front().getTime(),
                    queue_for_merge[index].front());
      queue_for_merge[index].pop();
    } else {
      break;
    }
  }
  for (size_t i = 0; i < bags_path.size(); ++i) {
    bags[i].close();
  }
  bag_out.close();
}