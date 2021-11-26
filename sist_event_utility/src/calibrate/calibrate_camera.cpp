#include <boost/foreach.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <Eigen/Eigen>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sist_event_utility/EventArray.h>

#include <map>
#include <string>
#include <vector>

#define foreach BOOST_FOREACH
#define NO_GT_ERROR -101
#define SUCC 0
#define KINECT_TIME 0.04

using EventArray = sist_event_utility::EventArray;
using MarkerCv   = std::pair<std::vector<int>, std::vector<cv::Point2f>>;
static bool SELECT;
static bool SAVE_IMAGE;

cv::Point2d Projection(const cv::Point3d& obj, const cv::Mat& K, const Eigen::Affine3d& extrinsic) {
  Eigen::Vector3d obj_p(obj.x, obj.y, obj.z);
  // obj_p = extrinsic * obj_p;
  obj_p = extrinsic.rotation() * obj_p;
  obj_p += extrinsic.translation();
  obj_p    = obj_p / obj_p.z();
  double u = obj_p.x() * K.at<double>(0, 0) + K.at<double>(0, 2);
  double v = obj_p.y() * K.at<double>(1, 1) + K.at<double>(1, 2);
  return cv::Point2d(u, v);
}

class CostFunctionPnP {
 private:
  cv::Point3d object_point_;
  cv::Point2d image_point_;
  cv::Mat K_;
  Eigen::Quaterniond q_gt_body_world_;
  Eigen::Vector3d t_gt_body_world_;

 public:
  CostFunctionPnP(const cv::Point3d& objpoint,
                  const cv::Point2f& imgpoint,
                  const cv::Mat camera_intrinsic,
                  const Eigen::Affine3d& gt)
      : object_point_(objpoint), K_(camera_intrinsic) {
    q_gt_body_world_ = gt.inverse().rotation();
    t_gt_body_world_ = gt.inverse().translation();
    image_point_.x   = imgpoint.x;
    image_point_.y   = imgpoint.y;
  }

  template <typename T>
  bool operator()(const T* const q_cam_body,
                  const T* const t_cam_body,
                  const T* const q_world_board,
                  const T* const t_world_board,
                  T* residual) const {
    T point_board[3];
    T point_world[3];
    T point_body[3];
    T point_camera[3];
    T q_body_world[4];
    T body_world[3];

    q_body_world[0] = T(q_gt_body_world_.w());
    q_body_world[1] = T(q_gt_body_world_.x());
    q_body_world[2] = T(q_gt_body_world_.y());
    q_body_world[3] = T(q_gt_body_world_.z());

    point_board[0] = T(object_point_.x);
    point_board[1] = T(object_point_.y);
    point_board[2] = T(object_point_.z);
    ceres::QuaternionRotatePoint(q_world_board, point_board, point_world);
    point_world[0] += t_world_board[0];
    point_world[1] += t_world_board[1];
    point_world[2] += t_world_board[2];
    // std::cout << point_world[0];

    ceres::QuaternionRotatePoint(q_body_world, point_world, point_body);
    point_body[0] += T(t_gt_body_world_.x());
    point_body[0] += T(t_gt_body_world_.y());
    point_body[0] += T(t_gt_body_world_.z());

    ceres::QuaternionRotatePoint(q_cam_body, point_body, point_camera);
    point_camera[0] += t_cam_body[0];
    point_camera[1] += t_cam_body[1];
    point_camera[2] += t_cam_body[2];

    T x = point_camera[0] / point_camera[2];
    T y = point_camera[1] / point_camera[2];

    T u = x * K_.at<double>(0, 0) + K_.at<double>(0, 2);
    T v = y * K_.at<double>(1, 1) + K_.at<double>(1, 2);

    T u_img = T(image_point_.x);
    T v_img = T(image_point_.y);

    // std::cout << u_img << std::endl
    //           << u << std::endl
    //           << v << std::endl
    //           << K_.at<double>(0, 0) << std::endl
    //           << x << std::endl;
    residual[0] = u - u_img;
    residual[1] = v - v_img;

    return true;
  }
  static ceres::CostFunction* create(const cv::Point3d objpoint,
                                     const cv::Point2f imgpoint,
                                     const cv::Mat camera_intrinsic,
                                     const Eigen::Affine3d& gt) {
    return new ceres::AutoDiffCostFunction<CostFunctionPnP, 2, 4, 3, 4, 3>(
        new CostFunctionPnP(objpoint, imgpoint, camera_intrinsic, gt));
  }
};

struct SyncFrame
{
  using Ptr = boost::shared_ptr<SyncFrame>;
  cv::Mat rbg_1_;
  cv::Mat rbg_2_;
  cv::Mat rbg_3_;
  cv::Mat rbg_4_;
  cv::Mat rbg_5_;
  cv::Mat ev_1_;
  cv::Mat ev_2_;

  cv::Mat rbg_1_rectified_;
  cv::Mat rbg_2_rectified_;
  cv::Mat rbg_3_rectified_;
  cv::Mat rbg_4_rectified_;
  cv::Mat rbg_5_rectified_;
  cv::Mat ev_1_rectified_;
  cv::Mat ev_2_rectified_;

  MarkerCv rbg_1_marker_;
  MarkerCv rbg_2_marker_;
  MarkerCv rbg_3_marker_;
  MarkerCv rbg_4_marker_;
  MarkerCv rbg_5_marker_;
  MarkerCv ev_1_marker_;
  MarkerCv ev_2_marker_;

  std::vector<sensor_msgs::Imu> imu_massages;
  Eigen::Affine3d gt;
  bool valid_ = true;
};

inline bool Near(double a, double b) {
  return a - b < 1e-3 && b - a < 1e-3;
}

int LoadImageFromEvent(std::vector<sist_event_utility::EventArray::Ptr>& event,
                       const ros::Time time,
                       cv::Mat& mat) {
  if (event.front()->header.stamp > time) return -1;
  auto evnet_strat = std::lower_bound(
      event.begin(), event.end(), time.toSec() - 0.01, [](EventArray::Ptr a, double b) {
        return a->header.stamp.toSec() < b;
      });

  mat = cv::Mat(event.front()->height, event.front()->width, CV_8UC3, cv::Scalar::all(255));
  while (evnet_strat < event.end() && (*evnet_strat)->header.stamp.toSec() < time.toSec() + 0.01) {
    for (const auto& e : (*evnet_strat)->events) {
      mat.at<cv::Vec3b>(e.y, e.x)[0] = std::max(mat.at<cv::Vec3b>(e.y, e.x)[0] - 83, 0);
      mat.at<cv::Vec3b>(e.y, e.x)[1] = std::max(mat.at<cv::Vec3b>(e.y, e.x)[1] - 83, 0);
      mat.at<cv::Vec3b>(e.y, e.x)[2] = std::max(mat.at<cv::Vec3b>(e.y, e.x)[2] - 83, 0);
    }
    evnet_strat++;
  }
  event.erase(event.begin(), evnet_strat);
  return 0;
}

int DetectAndSaveImage(cv::Mat& image,
                       cv::Mat& image_raw,
                       MarkerCv& out_marker,
                       const std::string& raw_path,
                       bool& valid) {
  auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  auto board      = cv::aruco::CharucoBoard::create(8, 4, 0.04f, 0.03f, dictionary);
  auto params     = cv::aruco::DetectorParameters::create();
  if (SAVE_IMAGE) {
    cv::imwrite(raw_path + "raw.png", image_raw);
    cv::imwrite(raw_path + "distorted.png", image);
  }
  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners;
  cv::aruco::detectMarkers(image, board->dictionary, markerCorners, markerIds, params);
  if (markerIds.size() > 0) {
    cv::aruco::drawDetectedMarkers(image, markerCorners, markerIds);
    cv::aruco::interpolateCornersCharuco(
        markerCorners, markerIds, image, board, out_marker.second, out_marker.first);
    if (out_marker.first.size() > 0)
      cv::aruco::drawDetectedCornersCharuco(
          image, out_marker.second, out_marker.first, cv::Scalar(255, 0, 255));
  }
  if (SELECT) {
    cv::imshow("Frame choose", image);
    char k = cv::waitKey(0);
    if (k == 'f') valid = false;
  }
  if (SAVE_IMAGE) cv::imwrite(raw_path + "detected.png", image);
  return SUCC;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "calibration");
  ros::NodeHandle nh;

  ///////////////
  std::vector<cv::Mat> camera_intrinsic{cv::Mat::zeros(3, 3, CV_64F),
                                        cv::Mat::zeros(3, 3, CV_64F),
                                        cv::Mat::zeros(3, 3, CV_64F),
                                        cv::Mat::zeros(3, 3, CV_64F),
                                        cv::Mat::zeros(3, 3, CV_64F)};
  std::vector<cv::Mat> distortion_coeffs{cv::Mat::zeros(5, 1, CV_64F),
                                         cv::Mat::zeros(5, 1, CV_64F),
                                         cv::Mat::zeros(5, 1, CV_64F),
                                         cv::Mat::zeros(5, 1, CV_64F),
                                         cv::Mat::zeros(5, 1, CV_64F)};

  camera_intrinsic[0].at<double>(0, 0) = 890.55862;
  camera_intrinsic[0].at<double>(1, 1) = 891.05305;
  camera_intrinsic[0].at<double>(2, 2) = 1.000000;
  camera_intrinsic[0].at<double>(0, 2) = 629.38333;
  camera_intrinsic[0].at<double>(1, 2) = 539.59751;

  camera_intrinsic[1].at<double>(0, 0) = 893.64448;
  camera_intrinsic[1].at<double>(1, 1) = 893.84469;
  camera_intrinsic[1].at<double>(2, 2) = 1.000000;
  camera_intrinsic[1].at<double>(0, 2) = 612.48343;
  camera_intrinsic[1].at<double>(1, 2) = 529.01122;

  camera_intrinsic[2].at<double>(0, 0) = 602.965;
  camera_intrinsic[2].at<double>(1, 1) = 602.786;
  camera_intrinsic[2].at<double>(2, 2) = 1.000000;
  camera_intrinsic[2].at<double>(0, 2) = 640.028;
  camera_intrinsic[2].at<double>(1, 2) = 364.642;

  camera_intrinsic[3].at<double>(0, 0) = 329.94156;
  camera_intrinsic[3].at<double>(1, 1) = 329.90821;
  camera_intrinsic[3].at<double>(2, 2) = 1.000000;
  camera_intrinsic[3].at<double>(0, 2) = 309.09818;
  camera_intrinsic[3].at<double>(1, 2) = 236.25427;

  camera_intrinsic[4].at<double>(0, 0) = 327.15149;
  camera_intrinsic[4].at<double>(1, 1) = 327.11971;
  camera_intrinsic[4].at<double>(2, 2) = 1.000000;
  camera_intrinsic[4].at<double>(0, 2) = 319.06411;
  camera_intrinsic[4].at<double>(1, 2) = 231.65257;

  distortion_coeffs[0].at<double>(0, 0) = -0.314092;
  distortion_coeffs[0].at<double>(1, 0) = 0.097589;
  distortion_coeffs[0].at<double>(2, 0) = -0.000345;
  distortion_coeffs[0].at<double>(3, 0) = -0.000504;
  distortion_coeffs[0].at<double>(4, 0) = 0.000000;

  distortion_coeffs[1].at<double>(0, 0) = -0.320961;
  distortion_coeffs[1].at<double>(1, 0) = 0.106100;
  distortion_coeffs[1].at<double>(2, 0) = 0.000021;
  distortion_coeffs[1].at<double>(3, 0) = 0.000430;
  distortion_coeffs[1].at<double>(4, 0) = 0.000000;

  distortion_coeffs[2].at<double>(0, 0) = 0;
  distortion_coeffs[2].at<double>(1, 0) = 0;
  distortion_coeffs[2].at<double>(2, 0) = 0;
  distortion_coeffs[2].at<double>(3, 0) = 0;
  distortion_coeffs[2].at<double>(4, 0) = 0;

  distortion_coeffs[3].at<double>(0, 0) = -0.028820;
  distortion_coeffs[3].at<double>(1, 0) = 0.039504;
  distortion_coeffs[3].at<double>(2, 0) = 0.000046;
  distortion_coeffs[3].at<double>(3, 0) = 0.000361;
  distortion_coeffs[3].at<double>(4, 0) = 0.000000;

  distortion_coeffs[4].at<double>(0, 0) = -0.029314;
  distortion_coeffs[4].at<double>(1, 0) = 0.039597;
  distortion_coeffs[4].at<double>(2, 0) = -0.000583;
  distortion_coeffs[4].at<double>(3, 0) = 0.000953;
  distortion_coeffs[4].at<double>(4, 0) = 0.000000;

  std::vector<std::pair<cv::Mat, cv::Mat>> undistort_maps;
  std::vector<cv::Mat> projection_matrixs{cv::Mat::zeros(3, 3, CV_64F),
                                          cv::Mat::zeros(3, 3, CV_64F),
                                          cv::Mat::zeros(3, 3, CV_64F),
                                          cv::Mat::zeros(3, 3, CV_64F),
                                          cv::Mat::zeros(3, 3, CV_64F)};

  std::vector<cv::Size> camera_sizes{cv::Size(1224, 1024),
                                     cv::Size(1224, 1024),
                                     cv::Size(1280, 720),
                                     cv::Size(640, 480),
                                     cv::Size(640, 480)};

  cv::Mat rectify_matrix = cv::Mat::zeros(3, 3, CV_64F);

  rectify_matrix.at<double>(0, 0) = 1;
  rectify_matrix.at<double>(1, 1) = 1;
  rectify_matrix.at<double>(2, 2) = 1;

  for (int i = 0; i < 5; ++i) {
    undistort_maps.push_back(std::pair<cv::Mat, cv::Mat>());
    undistort_maps[i].first  = cv::Mat();
    undistort_maps[i].second = cv::Mat();
    projection_matrixs[i]    = cv::getOptimalNewCameraMatrix(
        camera_intrinsic[i], distortion_coeffs[i], camera_sizes[i], 0, camera_sizes[i]);
    cv::initUndistortRectifyMap(camera_intrinsic[i],
                                distortion_coeffs[i],
                                rectify_matrix,
                                projection_matrixs[i],
                                camera_sizes[i],
                                CV_32F,
                                undistort_maps[i].first,
                                undistort_maps[i].second);
  }

  ///////////////

  std::string topic_ev_1;
  std::string topic_ev_2;
  std::string topic_rgb_1;
  std::string topic_rgb_2;
  std::string topic_rgb_3;
  std::string topic_rgb_4;
  std::string topic_rgb_5;
  std::string topic_imu;
  std::string topic_gt;

  std::string raw_image_path;
  std::string select_image_path;
  std::string bag_path;
  int frame_gt_ratio;
  double view_size_ratio;

  // rgb1 is required
  ros::param::get("topic_rgb_1", topic_rgb_1);
  bool rgb_2 = ros::param::get("topic_rgb_2", topic_rgb_2);
  bool rgb_3 = ros::param::get("topic_rgb_3", topic_rgb_3);
  bool rgb_4 = ros::param::get("topic_rgb_4", topic_rgb_4);
  bool rgb_5 = ros::param::get("topic_rgb_5", topic_rgb_5);
  bool ev_1  = ros::param::get("topic_ev_1", topic_ev_1);
  bool ev_2  = ros::param::get("topic_ev_2", topic_ev_2);
  bool imu   = ros::param::get("topic_imu", topic_imu);
  ros::param::get("topic_gt", topic_gt);

  ros::param::get("selected", SELECT);
  ros::param::get("save_image", SAVE_IMAGE);
  ros::param::get("raw_image_path", raw_image_path);
  ros::param::get("select_image_path", select_image_path);

  ros::param::get("bag_path", bag_path);
  ros::param::get("view_size_ratio", view_size_ratio);
  bool gt_sync = ros::param::get("frame_gt_ratio", frame_gt_ratio);

  std::deque<sensor_msgs::Image::Ptr> rgb_1_buffer;
  std::deque<sensor_msgs::Image::Ptr> rgb_2_buffer;
  std::deque<sensor_msgs::Image::Ptr> rgb_3_buffer;
  std::deque<sensor_msgs::Image::Ptr> rgb_4_buffer;
  std::deque<sensor_msgs::Image::Ptr> rgb_5_buffer;
  std::vector<sist_event_utility::EventArray::Ptr> ev_1_buffer;
  std::vector<sist_event_utility::EventArray::Ptr> ev_2_buffer;

  std::deque<sensor_msgs::Imu::Ptr> imu_buffer;
  std::deque<geometry_msgs::PoseStamped::Ptr> gt_buffer;

  std::vector<SyncFrame::Ptr> sync_frames{};

  rosbag::Bag bag;
  bag.open(bag_path, rosbag::bagmode::Read);
  rosbag::View view(bag);
  ROS_INFO("Start loading ...");

  auto current_time = ros::Time(0);
  Eigen::Affine3d pose_pre, pose_late, pose_temp, pose_diff;
  bool first_gt = true;
  ros::Time kinect_last_time(0);
  foreach (rosbag::MessageInstance const m, view) {
    if (m.getTopic() == topic_rgb_1) {
      rgb_1_buffer.emplace_back(m.instantiate<sensor_msgs::Image>());
      if (current_time.toNSec() == 0) current_time = m.getTime();
    }
    if (rgb_2 && m.getTopic() == topic_rgb_2)
      rgb_2_buffer.emplace_back(m.instantiate<sensor_msgs::Image>());
    if (rgb_3 && m.getTopic() == topic_rgb_3)
      rgb_3_buffer.emplace_back(m.instantiate<sensor_msgs::Image>());
    if (rgb_4 && m.getTopic() == topic_rgb_4)
      rgb_4_buffer.emplace_back(m.instantiate<sensor_msgs::Image>());
    if (rgb_5 && m.getTopic() == topic_rgb_5) {
      if (kinect_last_time.toNSec() != 0 &&
          m.getTime().toSec() - kinect_last_time.toSec() > KINECT_TIME) {
        rgb_3_buffer.emplace_back(sensor_msgs::ImagePtr());
        ROS_WARN("Detect one frame lost");
      }
      kinect_last_time = m.getTime();
      rgb_5_buffer.emplace_back(m.instantiate<sensor_msgs::Image>());
    }

    if (ev_1 && m.getTopic() == topic_ev_1)
      ev_1_buffer.emplace_back(m.instantiate<sist_event_utility::EventArray>());
    if (ev_2 && m.getTopic() == topic_ev_2)
      ev_2_buffer.emplace_back(m.instantiate<sist_event_utility::EventArray>());

    if (imu && m.getTopic() == topic_imu)
      imu_buffer.emplace_back(m.instantiate<sensor_msgs::Imu>());
    if (m.getTopic() == topic_gt) {
      gt_buffer.emplace_back(m.instantiate<geometry_msgs::PoseStamped>());
      if (first_gt) {
        pose_temp = Eigen::Translation3d(gt_buffer.front()->pose.position.x,
                                         gt_buffer.front()->pose.position.y,
                                         gt_buffer.front()->pose.position.z) *
                    Eigen::Quaterniond(gt_buffer.front()->pose.orientation.w,
                                       gt_buffer.front()->pose.orientation.x,
                                       gt_buffer.front()->pose.orientation.y,
                                       gt_buffer.front()->pose.orientation.z);
        pose_late = pose_temp;
      }
    }
    while (rgb_1_buffer.size() > 10) {
      pose_pre  = pose_temp;
      pose_temp = pose_late;
      for (int i = 0; i < frame_gt_ratio; ++i) {
        gt_buffer.pop_front();
      }
      pose_late = Eigen::Translation3d(gt_buffer.front()->pose.position.x,
                                       gt_buffer.front()->pose.position.y,
                                       gt_buffer.front()->pose.position.z) *
                  Eigen::Quaterniond(gt_buffer.front()->pose.orientation.w,
                                     gt_buffer.front()->pose.orientation.x,
                                     gt_buffer.front()->pose.orientation.y,
                                     gt_buffer.front()->pose.orientation.z);

      pose_diff = pose_pre.inverse() * pose_late;
      Eigen::AngleAxisd rot(pose_diff.rotation());
      if (pose_diff.translation().norm() < 0.0015 && rot.angle() < 0.0015) {
        auto load = boost::make_shared<SyncFrame>();
        if (ev_1) {
          LoadImageFromEvent(ev_1_buffer, rgb_1_buffer.front()->header.stamp, load->ev_1_);
        }
        if (ev_2) {
          LoadImageFromEvent(ev_2_buffer, rgb_1_buffer.front()->header.stamp, load->ev_2_);
        }
        load->rbg_1_ =
            cv_bridge::toCvCopy(*rgb_1_buffer.front(), sensor_msgs::image_encodings::BGR8)->image;
        if (rgb_2) {
          load->rbg_2_ =
              cv_bridge::toCvCopy(*rgb_2_buffer.front(), sensor_msgs::image_encodings::BGR8)->image;
        }
        if (rgb_3) {
          load->rbg_3_ =
              cv_bridge::toCvCopy(*rgb_3_buffer.front(), sensor_msgs::image_encodings::BGR8)->image;
        }
        if (rgb_4) {
          load->rbg_4_ =
              cv_bridge::toCvCopy(*rgb_4_buffer.front(), sensor_msgs::image_encodings::BGR8)->image;
        }
        if (rgb_5) {
          if (rgb_5_buffer.front() != nullptr) {
            load->rbg_5_ =
                cv_bridge::toCvCopy(*rgb_5_buffer.front(), sensor_msgs::image_encodings::BGR8)
                    ->image;
          }
        }
        load->gt = pose_temp;

        sync_frames.emplace_back(load);
      }
      rgb_1_buffer.pop_front();
      if (rgb_2) rgb_2_buffer.pop_front();
      if (rgb_3) rgb_3_buffer.pop_front();
      if (rgb_4) rgb_4_buffer.pop_front();
      if (rgb_5) rgb_5_buffer.pop_front();
      ROS_DEBUG_STREAM("Time diff" << rgb_1_buffer.front()->header.stamp.toSec() -
                                          gt_buffer.front()->header.stamp.toSec()
                                   << "s");
    }
  }
  ROS_INFO_STREAM("There is " << sync_frames.size() << " frame");
  ROS_INFO_STREAM("Start Undistorting");
  for (auto&& frame : sync_frames) {
    if (frame->rbg_5_.empty()) {
      frame->valid_ = false;
      continue;
    }
    cv::remap(frame->rbg_1_,
              frame->rbg_1_rectified_,
              undistort_maps[0].first,
              undistort_maps[0].second,
              cv::INTER_LINEAR);
    cv::remap(frame->rbg_2_,
              frame->rbg_2_rectified_,
              undistort_maps[1].first,
              undistort_maps[1].second,
              cv::INTER_LINEAR);
    cv::remap(frame->rbg_5_,
              frame->rbg_5_rectified_,
              undistort_maps[2].first,
              undistort_maps[2].second,
              cv::INTER_LINEAR);
    cv::remap(frame->ev_1_,
              frame->ev_1_rectified_,
              undistort_maps[3].first,
              undistort_maps[4].second,
              cv::INTER_LINEAR);
    cv::remap(frame->ev_2_,
              frame->ev_2_rectified_,
              undistort_maps[4].first,
              undistort_maps[4].second,
              cv::INTER_LINEAR);
  }
  ROS_INFO_STREAM("Finish Undistorting");

  ROS_INFO_STREAM("Start Detection");
  for (int i = 0; i < sync_frames.size(); ++i) {
    if (sync_frames[i]->rbg_5_.empty()) continue;

    std::string index_ = std::to_string(i);
    cv::Mat img;
    img        = cv::Mat(2160, 2504, CV_8UC3, cv::Scalar::all(0));
    cv::Mat r1 = img(cv::Rect(0, 0, 1224, 1024));
    cv::Mat r2 = img(cv::Rect(0, 1024, 1224, 1024));
    cv::Mat r5 = img(cv::Rect(1224, 0, 1280, 720));
    cv::Mat e1 = img(cv::Rect(1224, 720, 960, 720));
    cv::Mat e2 = img(cv::Rect(1224, 1440, 960, 720));
    cv::Mat e1_temp, e2_temp;
    sync_frames[i]->rbg_1_rectified_.copyTo(r1);
    sync_frames[i]->rbg_2_rectified_.copyTo(r2);
    sync_frames[i]->rbg_5_rectified_.copyTo(r5);
    sync_frames[i]->ev_1_rectified_.copyTo(e1_temp);
    sync_frames[i]->ev_2_rectified_.copyTo(e2_temp);
    DetectAndSaveImage(r1,
                       sync_frames[i]->rbg_1_,
                       sync_frames[i]->rbg_1_marker_,
                       raw_image_path + index_ + "_r1",
                       sync_frames[i]->valid_);
    DetectAndSaveImage(r2,
                       sync_frames[i]->rbg_2_,
                       sync_frames[i]->rbg_2_marker_,
                       raw_image_path + index_ + "_r2",
                       sync_frames[i]->valid_);
    DetectAndSaveImage(r5,
                       sync_frames[i]->rbg_5_,
                       sync_frames[i]->rbg_5_marker_,
                       raw_image_path + index_ + "_r5",
                       sync_frames[i]->valid_);
    DetectAndSaveImage(e1_temp,
                       sync_frames[i]->ev_1_,
                       sync_frames[i]->ev_1_marker_,
                       raw_image_path + index_ + "_e1",
                       sync_frames[i]->valid_);
    DetectAndSaveImage(e2_temp,
                       sync_frames[i]->ev_2_,
                       sync_frames[i]->ev_2_marker_,
                       raw_image_path + index_ + "_e2",
                       sync_frames[i]->valid_);

    cv::resize(e1_temp, e1, cv::Size(960, 720));
    cv::resize(e2_temp, e2, cv::Size(960, 720));
    if (SELECT) {
      cv::Mat image_mini;
      cv::resize(
          img, image_mini, cv::Size(img.size[1] / view_size_ratio, img.size[0] / view_size_ratio));
      cv::imshow("Frame choose", image_mini);
      char k = cv::waitKey(0);
      if (k == 'f') sync_frames[i]->valid_ = false;
      if (k == 0x1b) return -1;
    }
  }
  std::vector<SyncFrame::Ptr> sync_frames_selected{};
  for (auto frame : sync_frames) {
    if (frame->valid_) sync_frames_selected.emplace_back(frame);
  }
  sync_frames.clear();
  ROS_INFO_STREAM("Finish Detection");
  ROS_INFO_STREAM("Finally Get " << sync_frames_selected.size() << " Frames");

  ////////
  ROS_INFO_STREAM("Start Optimize");
  ceres::Problem problem;

  std::vector<double> q_world_board_vector{1, 0, 0, 0};
  std::vector<double> t_world_board_vector{0, 0, 1};
  double* q_w_board = q_world_board_vector.data();
  double* t_w_board = t_world_board_vector.data();
  problem.AddParameterBlock(q_w_board, 4, new ceres::QuaternionParameterization());
  problem.AddParameterBlock(t_w_board, 3);

  std::vector<double> unit_t{0, 0, 1};
  std::vector<double> q_cam_body_vector;
  std::vector<double> t_cam_body_vector;
  for (int index_cx = 0; index_cx < 5; index_cx++) {
    q_cam_body_vector.insert(
        q_cam_body_vector.end(), q_world_board_vector.begin(), q_world_board_vector.end());
    t_cam_body_vector.insert(
        t_cam_body_vector.end(), t_world_board_vector.begin(), t_world_board_vector.end());
    double* param_q = q_cam_body_vector.data() + (index_cx * 4);
    double* param_t = t_cam_body_vector.data() + (index_cx * 3);
    problem.AddParameterBlock(param_q, 4, new ceres::QuaternionParameterization());
    problem.AddParameterBlock(param_t, 3);
  }

  std::map<int, cv::Point3d> rgb_map{{0, cv::Point3d(0.000, 0.122, 0.000)},
                                     {1, cv::Point3d(0.061, 0.122, 0.000)},
                                     {2, cv::Point3d(0.122, 0.122, 0.000)},
                                     {7, cv::Point3d(0.000, 0.061, 0.000)},
                                     {8, cv::Point3d(0.061, 0.061, 0.000)},
                                     {9, cv::Point3d(0.122, 0.061, 0.000)},
                                     {14, cv::Point3d(0.000, 0.000, 0.000)},
                                     {15, cv::Point3d(0.061, 0.000, 0.000)},
                                     {16, cv::Point3d(0.122, 0.000, 0.000)}};

  std::map<int, cv::Point3d> ev_map{{4, cv::Point3d(0.244, 0.122, 0.000)},
                                    {5, cv::Point3d(0.305, 0.122, 0.000)},
                                    {6, cv::Point3d(0.366, 0.122, 0.000)},
                                    {11, cv::Point3d(0.244, 0.061, 0.000)},
                                    {12, cv::Point3d(0.305, 0.061, 0.000)},
                                    {13, cv::Point3d(0.366, 0.061, 0.000)},
                                    {18, cv::Point3d(0.244, 0.000, 0.000)},
                                    {19, cv::Point3d(0.305, 0.000, 0.000)},
                                    {20, cv::Point3d(0.366, 0.000, 0.000)}};
  double *q_c_body, *t_c_body;
  for (const auto frame : sync_frames_selected) {
    q_c_body = q_cam_body_vector.data();
    t_c_body = t_cam_body_vector.data();
    for (int i = 0; i < frame->rbg_1_marker_.first.size(); i++) {
      if (rgb_map.find(frame->rbg_1_marker_.first[i]) != rgb_map.end()) {
        problem.AddResidualBlock(
            CostFunctionPnP::create(rgb_map[frame->rbg_1_marker_.first[i]],
                                    frame->rbg_1_marker_.second[i],
                                    projection_matrixs[0],
                                    sync_frames_selected[0]->gt.inverse() * frame->gt),
            nullptr,
            q_c_body,
            t_c_body,
            q_w_board,
            t_w_board);
      }
    }

    q_c_body = q_cam_body_vector.data() + 4;
    t_c_body = t_cam_body_vector.data() + 3;
    for (int i = 0; i < frame->rbg_2_marker_.first.size(); i++) {
      if (rgb_map.find(frame->rbg_2_marker_.first[i]) != rgb_map.end()) {
        problem.AddResidualBlock(
            CostFunctionPnP::create(rgb_map[frame->rbg_2_marker_.first[i]],
                                    frame->rbg_2_marker_.second[i],
                                    projection_matrixs[1],
                                    sync_frames_selected[0]->gt.inverse() * frame->gt),
            nullptr,
            q_c_body,
            t_c_body,
            q_w_board,
            t_w_board);
      }
    }

    q_c_body = q_cam_body_vector.data() + 8;
    t_c_body = t_cam_body_vector.data() + 6;
    for (int i = 0; i < frame->rbg_5_marker_.first.size(); i++) {
      if (rgb_map.find(frame->rbg_5_marker_.first[i]) != rgb_map.end()) {
        problem.AddResidualBlock(
            CostFunctionPnP::create(rgb_map[frame->rbg_5_marker_.first[i]],
                                    frame->rbg_5_marker_.second[i],
                                    projection_matrixs[2],
                                    sync_frames_selected[0]->gt.inverse() * frame->gt),
            nullptr,
            q_c_body,
            t_c_body,
            q_w_board,
            t_w_board);
      }
    }

    q_c_body = q_cam_body_vector.data() + 12;
    t_c_body = t_cam_body_vector.data() + 9;
    for (int i = 0; i < frame->ev_1_marker_.first.size(); i++) {
      if (ev_map.find(frame->ev_1_marker_.first[i]) != ev_map.end()) {
        problem.AddResidualBlock(
            CostFunctionPnP::create(ev_map[frame->ev_1_marker_.first[i]],
                                    frame->ev_1_marker_.second[i],
                                    projection_matrixs[3],
                                    sync_frames_selected[0]->gt.inverse() * frame->gt),
            nullptr,
            q_c_body,
            t_c_body,
            q_w_board,
            t_w_board);
      }
    }

    q_c_body = q_cam_body_vector.data() + 16;
    t_c_body = t_cam_body_vector.data() + 12;
    for (int i = 0; i < frame->ev_2_marker_.first.size(); i++) {
      if (ev_map.find(frame->ev_2_marker_.first[i]) != ev_map.end()) {
        problem.AddResidualBlock(
            CostFunctionPnP::create(ev_map[frame->ev_2_marker_.first[i]],
                                    frame->ev_2_marker_.second[i],
                                    projection_matrixs[4],
                                    sync_frames_selected[0]->gt.inverse() * frame->gt),
            nullptr,
            q_c_body,
            t_c_body,
            q_w_board,
            t_w_board);
      }
    }
  }
  ceres::Solver::Options options;
  options.linear_solver_type           = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations           = 100;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::vector<Eigen::Affine3d> T_cam_body;
  Eigen::Affine3d T_world_board = Eigen::Translation3d(t_world_board_vector[0],
                                                       t_world_board_vector[1],
                                                       t_world_board_vector[2]) *
                                  Eigen::Quaterniond(q_world_board_vector[0],
                                                     q_world_board_vector[41],
                                                     q_world_board_vector[2],
                                                     q_world_board_vector[3])
                                      .normalized();
  for (int index_cx = 0; index_cx < 5; index_cx++) {
    T_cam_body.emplace_back(Eigen::Translation3d(t_cam_body_vector[3 * index_cx],
                                                 t_cam_body_vector[3 * index_cx + 1],
                                                 t_cam_body_vector[3 * index_cx + 2]) *
                            Eigen::Quaterniond(q_cam_body_vector[4 * index_cx],
                                               q_cam_body_vector[4 * index_cx + 1],
                                               q_cam_body_vector[4 * index_cx + 2],
                                               q_cam_body_vector[4 * index_cx + 3])
                                .normalized());
  }
  std::cout << "T_wb" << std::endl
            << "R:" << std::endl
            << T_world_board.rotation() << std::endl
            << "t:" << std::endl
            << T_world_board.translation() << std::endl
            << "m" << std::endl
            << std::endl;

  for (const auto frame : sync_frames_selected) {
    for (auto p : frame->rbg_1_marker_.first) {
      cv::Point3d point;
      if (rgb_map.find(p) != rgb_map.end()) point = rgb_map[p];
      cv::Point2d corner = Projection(point,
                                      projection_matrixs[0],
                                      T_cam_body[0] * frame->gt.inverse() *
                                          sync_frames_selected[0]->gt * T_world_board);
      cv::circle(frame->rbg_1_rectified_, corner, 2, (0, 255, 255), 3);
    }
    cv::imshow("test_win", frame->rbg_1_rectified_);
    cv::waitKey(0);
  }
  return 0;
}