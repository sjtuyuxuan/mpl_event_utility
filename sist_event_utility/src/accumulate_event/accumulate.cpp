#include <algorithm>
#include <cv_bridge/cv_bridge.h>
#include <deque>
#include <image_transport/image_transport.h>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sist_event_utility/Event.h>
#include <sist_event_utility/EventArray.h>
#include <string>

using Event      = sist_event_utility::Event;
using EventArray = sist_event_utility::EventArray;
using EventQueue = std::deque<Event>;

std::vector<EventArray::ConstPtr> queue_ev_left;
std::vector<EventArray::ConstPtr> queue_ev_right;
std::mutex lk_ev_left;
std::mutex lk_ev_right;

void eventCallbackLeft(const EventArray::ConstPtr& msg) {
  ROS_WARN_ONCE("Receive event messages");
  lk_ev_left.lock();
  queue_ev_left.push_back(msg);
  lk_ev_left.unlock();
}

void eventCallbackRight(const EventArray::ConstPtr& msg) {
  ROS_WARN_ONCE("Receive event messages");
  lk_ev_right.lock();
  queue_ev_right.push_back(msg);
  lk_ev_right.unlock();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "event_camera_calibration_bridge_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  std::string topic_ev_left;
  std::string topic_ev_right;
  std::string topic_out_left;
  std::string topic_out_right;

  float acc_rate;
  int height_ev;
  int width_ev;
  int n_threads;
  double acc_duration;
  bool polarity_difference;

  ros::param::get("topic_ev_left", topic_ev_left);
  ros::param::get("topic_ev_right", topic_ev_right);
  ros::param::get("topic_out_left", topic_out_left);
  ros::param::get("topic_out_right", topic_out_right);

  ros::param::get("acc_rate", acc_rate);
  ros::param::get("height_ev", height_ev);
  ros::param::get("width_ev", width_ev);
  ros::param::get("n_threads", n_threads);
  bool control_duration = ros::param::get("acc_duration", acc_duration);
  ros::param::get("polarity_difference", polarity_difference);

  image_transport::Publisher pub1 = it.advertise(topic_out_left, 30);
  image_transport::Publisher pub2 = it.advertise(topic_out_right, 30);
  ros::Subscriber sub1 =
      nh.subscribe<EventArray>(topic_ev_left, 100000, eventCallbackLeft);
  ros::Subscriber sub2 =
      nh.subscribe<EventArray>(topic_ev_right, 100000, eventCallbackRight);

  ros::Rate r(acc_rate);

  while (ros::ok()) {
    std::vector<EventArray::ConstPtr> event_vec_left;
    std::vector<EventArray::ConstPtr> event_vec_right;
    cv_bridge::CvImage acc_events_left;
    cv_bridge::CvImage acc_events_right;

    if (polarity_difference) {
      acc_events_left.image =
          cv::Mat(height_ev, width_ev, CV_8U, cv::Scalar(127));
      acc_events_right.image =
          cv::Mat(height_ev, width_ev, CV_8U, cv::Scalar(127));
    } else {
      acc_events_left.image =
          cv::Mat(height_ev, width_ev, CV_8U, cv::Scalar(255));
      acc_events_right.image =
          cv::Mat(height_ev, width_ev, CV_8U, cv::Scalar(255));
    }
    lk_ev_left.lock();
    event_vec_left.swap(queue_ev_left);
    lk_ev_left.unlock();

    lk_ev_right.lock();
    event_vec_right.swap(queue_ev_right);
    lk_ev_right.unlock();

    if (event_vec_left.size() == 0 || event_vec_right.size() == 0) {
      r.sleep();
      ros::spinOnce();
      continue;
    }

    if (control_duration) {
      ros::Time t_left = event_vec_left.front()->header.stamp;
      auto it_left_end = std::lower_bound(event_vec_left.begin(),
                                          event_vec_left.end(),
                                          t_left.toSec() + acc_duration / 1000,
                                          [](EventArray::ConstPtr a, double b) {
                                            return a->header.stamp.toSec() < b;
                                          });
      event_vec_left.resize(it_left_end - event_vec_left.begin());

      ros::Time t_right = event_vec_right.front()->header.stamp;
      auto it_right_end =
          std::lower_bound(event_vec_right.begin(),
                           event_vec_right.end(),
                           t_right.toSec() + acc_duration / 1000,
                           [](EventArray::ConstPtr a, double b) {
                             return a->header.stamp.toSec() < b;
                           });
      event_vec_right.resize(it_right_end - event_vec_right.begin());
    }

#pragma omp parallel for num_threads(n_threads)
    if (polarity_difference) {
      for (int j = 0; j < event_vec_left.size(); j++) {
        for (int i = 0; i < event_vec_left[j]->events.size(); i++) {
          if (event_vec_left[j]->events[i].polarity) {
            acc_events_left.image.at<uchar>(event_vec_left[j]->events[i].y,
                                            event_vec_left[j]->events[i].x) =
                std::min(acc_events_left.image.at<uint8_t>(
                             event_vec_left[j]->events[i].y,
                             event_vec_left[j]->events[i].x) +
                             63,
                         255);
          } else {
            acc_events_left.image.at<uchar>(event_vec_left[j]->events[i].y,
                                            event_vec_left[j]->events[i].x) =
                std::max(acc_events_left.image.at<uint8_t>(
                             event_vec_left[j]->events[i].y,
                             event_vec_left[j]->events[i].x) -
                             63,
                         0);
          }
        }
      }
    } else {
      for (int j = 0; j < event_vec_left.size(); j++) {
        for (int i = 0; i < event_vec_left[j]->events.size(); i++) {
          acc_events_left.image.at<uchar>(event_vec_left[j]->events[i].y,
                                          event_vec_left[j]->events[i].x) =
              std::max(acc_events_left.image.at<uint8_t>(
                           event_vec_left[j]->events[i].y,
                           event_vec_left[j]->events[i].x) -
                           83,
                       0);
        }
      }
    }

#pragma omp parallel for num_threads(n_threads)
    if (polarity_difference) {
      for (int j = 0; j < event_vec_right.size(); j++) {
        for (int i = 0; i < event_vec_right[j]->events.size(); i++) {
          if (event_vec_right[j]->events[i].polarity) {
            acc_events_right.image.at<uchar>(event_vec_right[j]->events[i].y,
                                             event_vec_right[j]->events[i].x) =
                std::min(acc_events_right.image.at<uint8_t>(
                             event_vec_right[j]->events[i].y,
                             event_vec_right[j]->events[i].x) +
                             63,
                         255);
          } else {
            acc_events_right.image.at<uchar>(event_vec_right[j]->events[i].y,
                                             event_vec_right[j]->events[i].x) =
                std::max(acc_events_right.image.at<uint8_t>(
                             event_vec_right[j]->events[i].y,
                             event_vec_right[j]->events[i].x) -
                             63,
                         0);
          }
        }
      }
    } else {
      for (int j = 0; j < event_vec_right.size(); j++) {
        for (int i = 0; i < event_vec_right[j]->events.size(); i++) {
          acc_events_right.image.at<uchar>(event_vec_right[j]->events[i].y,
                                           event_vec_right[j]->events[i].x) =
              std::max(acc_events_right.image.at<uint8_t>(
                           event_vec_right[j]->events[i].y,
                           event_vec_right[j]->events[i].x) -
                           83,
                       0);
        }
      }
    }

    acc_events_left.encoding     = "mono8";
    acc_events_left.header.stamp = ros::Time::now();
    pub1.publish(acc_events_left.toImageMsg());

    acc_events_right.encoding     = "mono8";
    acc_events_right.header.stamp = acc_events_left.header.stamp;
    pub2.publish(acc_events_right.toImageMsg());
    r.sleep();
    ros::spinOnce();
  }
  return 0;
}