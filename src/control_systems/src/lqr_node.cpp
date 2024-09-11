#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "LQRNode_lib.hpp"

using namespace std::chrono_literals;

class LQRNode : public rclcpp::Node
{
public:
  LQRNode()
  : Node("LQR_node"),horizon(1)
  {
    // subscriber1_ = this->create_subscription<geometry_msgs::msg::Twist>(
    //   "cmd_vel", 10, std::bind(&LQRNode::sub_callback, this, std::placeholders::_1));

    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odom>(
      "odom", 10, std::bind(&LQRNode::odom_callback, this, std::placeholders::_1));

    cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // timer_ = this->create_wall_timer(
    //   100ms, std::bind(&LQRNode::timer_callback, this));
    waypoint_ = (1,1);
    Q_ = (0,0,0
          0,0,0
          0,0,0);

    R_ = (0,0,0
          0,0,0);
    
  }

private:
  void odom_callback(const nav_msgs::msg::Odom::SharedPtr odom)
  {
    void getData();
    void controlLQR();
    void moveRobot();
  }

  void getData(const nav_msgs::msg::Odom::SharedPtr odom){

    yaw = odom->pose.pose.orientation.z;
    v = odom->twist.twist.linear.x;


  }
    

    LQR(Q,R,horizon);
    LQR.getA();
    LQR.getB();
    LQR.updateMatrices();
    LQR.computeRicatti();
    LQR.computeOptimalInput();
  }


    publisher_->publish(odom);
  

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  size_t count_;
  double x_, y_, theta_;
  double v_l, v_r;
  const double L = 1.0;  // Distance between wheels
  const double R = 0.5;  // Radius of the wheels
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LQRNode>());
  rclcpp::shutdown();
  return 0;
}
