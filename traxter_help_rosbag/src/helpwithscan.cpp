

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;


class scanRepublisher : public rclcpp::Node
{
public:
  // Initiate a Node called 'simple_subscriber'
  scanRepublisher()
  : Node("scan_republisher")
  {

    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan",1);
    bag_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
          "scan_out", 2, std::bind(&scanRepublisher::scan_topic_callback, this, _1));


    }


private:
  // Define a function called 'topic_callback' that receives a parameter named 'msg' 

void scan_topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){

    msg->header.stamp= this->get_clock()->now();;

    publisher_->publish(*msg);
}


  

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr bag_subscription_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  };



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Create a loop that will keep the program in execution
  rclcpp::spin(std::make_shared<scanRepublisher>());
  rclcpp::shutdown();
  return 0;
}

