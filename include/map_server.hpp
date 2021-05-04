#ifndef MAP_SERVER_HPP
#define MAP_SERVER_HPP

#include <string> 
#include <stdio.h> 
#include <cmath>
#include <iostream> 
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

// yaml-cpp
#include <yaml-cpp/yaml.h>

//OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "nav_msgs/msg/occupancy_grid.hpp"

#define OCC_GRID_UNKNOWN -1   //未知領域
#define OCC_GRID_FREE 0       //フリースペース
#define OCC_GRID_OCCUPIED 100 //占有領域

class MapServer : public rclcpp::Node
{
public:
  MapServer();

  void readImage(void);

  void readYaml(void);

  void publishMap(void);

  void timerCallback(void);
private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  cv::Mat map_image_;

  std::string image_filepath_;
  std::string yaml_filepath_;

  float resolution_; //map解像度[m/cell]
  double origin_x_;  //map原点X座標[m]
  double origin_y_;  //map原点Y座標[m]

  int width_;
  int height_;
};

#endif