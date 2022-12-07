

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using std::placeholders::_1;


class IMUHelper : public rclcpp::Node
{
public:
  // Initiate a Node called 'simple_subscriber'
  IMUHelper()
  : Node("imu_helper")
  {

    publisher_ = this->create_publisher<std_msgs::msg::Float32>("yawvalue",1);
    bag_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
          "imu", 2, std::bind(&IMUHelper::imu_topic_callback, this, _1));


    }


private:
  // Define a function called 'topic_callback' that receives a parameter named 'msg' 

void imu_topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg){

    tf2::Quaternion q(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    std_msgs::msg::Float32 outmsg;
    outmsg.data=yaw;

    publisher_->publish(outmsg);
}


  

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr bag_subscription_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  };



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Create a loop that will keep the program in execution
  rclcpp::spin(std::make_shared<IMUHelper>());
  rclcpp::shutdown();
  return 0;
}

