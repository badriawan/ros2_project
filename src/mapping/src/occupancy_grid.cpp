#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/header.hpp"




using namespace std::chrono_literals;

class OccupancyGridPub : public rclcpp::Node
{
public:
  OccupancyGridPub()
  : Node("occupancy_grid")
  {

    _publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/cmd_vel", 10);

    _timer = this->create_wall_timer(500ms, std::bind(&OccupancyGridPub::OGCallback, this));
  }

private:
  void OGCallback()
  {
    nav_msgs::msg::OccupancyGrid og_msg;

    og_msg.header = std_msgs::msg::Header();
    og_msg.header.stamp = this->now();
    og_msg.header.frame_id = "map_frame";

    og_msg.info.resolution = 1.0;
    og_msg.info.width = 3 ;
    og_msg.info.height = 3 ;
    
    std::vector<int8_t> data = {0,0,0,0,1,0,0,0,-1};
    og_msg.data = data;









    _publisher->publish(og_msg);

  }

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _publisher;  // Corrected type
  rclcpp::TimerBase::SharedPtr _timer;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyGridPub>());
  rclcpp::shutdown();
  return 0;
}
