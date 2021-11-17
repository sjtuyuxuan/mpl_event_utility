#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <string>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "blink_board");
  ros::NodeHandle nh;
  int half_width, height, pixel_per_block;
  ros::param::get("half_width", half_width);
  ros::param::get("height", height);
  ros::param::get("pixel_per_block", pixel_per_block);
  ROS_INFO_STREAM("We get parameter that"
                  << std::endl
                  << "half width is :" << half_width << std::endl
                  << "height is :" << height << std::endl
                  << "pixel_per_block is :" << pixel_per_block);
  auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  auto board      = cv::aruco::CharucoBoard::create(
      2 * half_width, height, 0.04f, 0.03f, dictionary);
  cv::Mat boardImage;
  board->draw(
      cv::Size(2 * half_width * pixel_per_block, height * pixel_per_block),
      boardImage,
      15,
      1);
  cv::Mat halfBoardImage = boardImage.clone();
  for (int w = half_width * pixel_per_block;
       w < 2 * half_width * pixel_per_block;
       ++w) {
    for (int h = 0; h < height * pixel_per_block; ++h) {
      halfBoardImage.at<uchar>(h, w) = 255;
    }
  }
  bool image_out;
  ros::param::get("image_out", image_out);
  if (image_out) {
    std::string path;
    ros::param::get("image_out_path", path);
    cv::imwrite(path + "full_charuco_board.png", boardImage);
    cv::imwrite(path + "part_charuco_board.png", halfBoardImage);
  }

  bool blink = true;
  cv::namedWindow("Blinking Charuco Board", cv::WINDOW_NORMAL);
  while (true) {
    if (blink) {
      cv::imshow("Blinking Charuco Board", boardImage);
      cv::waitKey(10);
      blink = false;
    } else {
      cv::imshow("Blinking Charuco Board", halfBoardImage);
      cv::waitKey(10);
      blink = true;
    }
  }
  cv::destroyAllWindows();
}
