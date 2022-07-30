

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>
#include <traxter_msgs/msg/light_imu.hpp>
#include "sensor_msgs/msg/imu.hpp"
using std::placeholders::_1;

class imuInterpreter : public rclcpp::Node
{
public:
  // Initiate a Node called 'simple_subscriber'
  imuInterpreter()
  : Node("imu_interpreter")
  {

    rclcpp::QoS qos(3);
    qos.keep_last(3);
    qos.best_effort();
    qos.durability_volatile();
    // Create a Subscriber object that will listen to the /counter topic and will call the 'topic_callback' function       // each time it reads something from the topic
    subscription_ = this->create_subscription<traxter_msgs::msg::LightImu>(
      "traxter/imu/data/unprocessed", qos, std::bind(&imuInterpreter::topic_callback, this, _1));

    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu",1);
    standard_imu_message.header.frame_id="imu_link";
  }

private:
  // Define a function called 'topic_callback' that receives a parameter named 'msg' 
  void topic_callback(const traxter_msgs::msg::LightImu::SharedPtr msg)
  {
    if(isFirst){
      biasQuat[0]=msg->q0 - 100;
      biasQuat[1]=msg->q1;
      biasQuat[2]=msg->q2;
      biasQuat[3]=msg->q3;
      isFirst=false;
    }else{
      standard_imu_message.orientation.x=(-msg->q1 + biasQuat[1])/100.0;
      standard_imu_message.orientation.y=(-msg->q2 + biasQuat[2])/100.0;
      standard_imu_message.orientation.z=(msg->q3 - biasQuat[3])/100.0;
      standard_imu_message.orientation.w=(msg->q0 - biasQuat[0])/100.0;
    }
    standard_imu_message.angular_velocity.x=msg->gyrox/100.0;
    standard_imu_message.angular_velocity.y=msg->gyroy/100.0;
    standard_imu_message.angular_velocity.z=msg->gyroz/100.0;
    standard_imu_message.linear_acceleration.x=msg->accx/100.0;
    standard_imu_message.linear_acceleration.y=msg->accy/100.0;
    standard_imu_message.linear_acceleration.z=msg->accz/100.0;
    standard_imu_message.header.stamp=this->now();

    publisher_->publish(standard_imu_message);
  }
  rclcpp::Subscription<traxter_msgs::msg::LightImu>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  sensor_msgs::msg::Imu standard_imu_message;
  bool isFirst=true;
  int biasQuat[4];
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Create a loop that will keep the program in execution
  rclcpp::spin(std::make_shared<imuInterpreter>());
  rclcpp::shutdown();
  return 0;
}

