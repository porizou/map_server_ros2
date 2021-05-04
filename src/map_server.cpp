#include "map_server.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

MapServer::MapServer() : Node("map_server_node")
{
  yaml_filepath_ = ament_index_cpp::get_package_share_directory("map_server_ros2") + "/map/map.yaml";
  std::cout << "YAML FILEPATH : " << yaml_filepath_ << std::endl;
  //yamlファイルの読み込み
  readYaml();
  //pgmイメージファイルの読み込み
  readImage();
	
  map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  timer_ = this->create_wall_timer(1s, std::bind(&MapServer::timerCallback, this));
}

void MapServer::readYaml(void)
{
  YAML::Node map = YAML::LoadFile(yaml_filepath_);

  image_filepath_ = map["image"].as<std::string>();
  resolution_ = map["resolution"].as<float>();
  origin_x_ = map["origin"][0].as<double>();
  origin_y_ = map["origin"][1].as<double>();

  std::cout << "IMAGE FILEPATH : " << image_filepath_ << std::endl;
  std::cout << "RESOLUTION [m/cell]: " << resolution_ << std::endl;
  std::cout << "ORIGIN X [m] : " << origin_x_ << std::endl;
  std::cout << "ORIGIN Y [m] : " << origin_y_ << std::endl;
}

void MapServer::readImage(void)
{
  //画像をグレースケールで読み込む
  map_image_ = cv::imread(image_filepath_,0);

  width_ = map_image_.size().width;
  height_ = map_image_.size().height;

  std::cout << "IMAGE WIDTH : " << width_ << std::endl;
  std::cout << "IMAGE HEIGHT : " << height_ << std::endl;
  //std::cout << map_image_ << std::endl;
}

void MapServer::publishMap(void)
{
  auto map_data = nav_msgs::msg::OccupancyGrid();
  map_data.header.frame_id = "/map";
  map_data.header.stamp = this -> now();

  map_data.info.resolution = 0.05;
  map_data.info.width = width_;
  map_data.info.height = height_;
  map_data.info.origin.position.x = origin_x_;
  map_data.info.origin.position.y = origin_y_;
  map_data.info.origin.position.z = 0.0;

  map_data.data.resize(map_data.info.width * map_data.info.height);

  for (size_t y = 0; y < map_data.info.height; y++) {
    for (size_t x = 0; x < map_data.info.width; x++) {
      int8_t map_cell;
      unsigned char pixel = map_image_.at<unsigned char>(y, x);
      if(pixel == 0)
      {
        map_cell = OCC_GRID_OCCUPIED;
      }
      else if(pixel == 205)
      {
        map_cell = OCC_GRID_UNKNOWN;
      }
      else if(pixel == 254)
      {
        map_cell = OCC_GRID_FREE;
      }
      else
      {
        map_cell = OCC_GRID_UNKNOWN;
      }
      //std::cout << map_cell << std::endl;
      map_data.data[map_data.info.width * (map_data.info.height - y - 1) + x] = map_cell;
    }
  }
  map_publisher_->publish(map_data);
}

void MapServer::timerCallback(void)
{
  publishMap();
}