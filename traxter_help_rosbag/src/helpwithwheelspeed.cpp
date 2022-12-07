#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>
#include <traxter_msgs/msg/dual_motor_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <math.h>
#include <cstdio>  // for EOF
#include <string>
#include <sstream>
#include <vector>
#include <limits.h>
using std::placeholders::_1;
using namespace std::chrono_literals;

class WheelSpeedHelper : public rclcpp::Node
{
public:

  WheelSpeedHelper()
  : Node("wheel_speed_helper")
  {
    rclcpp::QoS qos(3);
    qos.keep_last(3);
    qos.best_effort();
    qos.durability_volatile();

    ticks_subscription_ = this->create_subscription<traxter_msgs::msg::DualMotorArray>(
          "traxter/encoder/ticks", qos, std::bind(&WheelSpeedHelper::ticks_callback, this, _1));

    command_subscription_ = this->create_subscription<traxter_msgs::msg::DualMotorArray>(
          "traxter/motor/command", 1, std::bind(&WheelSpeedHelper::command_callback, this, _1));

    wheel_speed_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("traxter/wheel_speed/actual",1);
    wheel_command_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("traxter/wheel_speed/command",1);
    oldTime = this->now();
    outmsg.x=0.0;
    outmsg.y=0.0;
    outmsg.z=0.0;

  };
  private:

  void ticks_callback(const traxter_msgs::msg::DualMotorArray::SharedPtr msg)
  {

    newLticks= msg->motor1;
    newRticks= msg->motor2;
    status= msg->status;
    if(status==0){ //means de MD25 reset the encoders
      oldRticks=0;
      oldLticks=0;
    }
    if(isFirst){
      oldRticks=newRticks;
      oldLticks=newLticks;
      isFirst=false;
      oldTime = this->now();      
    }

    deltaEncoderTicks();
    newTime = this->now();

    double deltaT=(newTime - oldTime).nanoseconds()*1e-9;
    if (deltaT>0.0001)
    {
          double leftWheelSpeed=(deltaLeftTicks/deltaT) *2*PI / TICKSPERWHEELREV;
          double rightWheelSpeed=(deltaRightTicks/deltaT) *2*PI / TICKSPERWHEELREV;
          outmsg.x=leftWheelSpeed;
          outmsg.y=rightWheelSpeed;
          outmsg.z=double(newTime.nanoseconds()*1e-9);
          wheel_speed_publisher_->publish(outmsg);
    }
    oldTime=newTime;

  };

  void command_callback(const traxter_msgs::msg::DualMotorArray::SharedPtr msg){

    geometry_msgs::msg::Point comout;
    comout.x= (msg->motor1)/100.0;
    comout.y= (msg->motor2)/100.0;
    rclcpp::Time rightnow;
    rightnow=this->now();
    comout.z=double(rightnow.nanoseconds()*1e-9);
    wheel_command_publisher_->publish(comout);

  };


  void deltaEncoderTicks(){

    bool encoderFailure=false;

    deltaLeftTicks = (newLticks- oldLticks);
    deltaRightTicks = (newRticks- oldRticks);

    if (deltaLeftTicks > 1000000){

      encoderFailure=true;
      deltaLeftTicks=imin + deltaLeftTicks;
    
    }else if (deltaLeftTicks < -1000000){

      encoderFailure=true;

      deltaLeftTicks = imax-deltaLeftTicks;
    }

    if (deltaRightTicks > 1000000){

      encoderFailure=true;

      deltaRightTicks=imin + deltaRightTicks;
    
    }else if (deltaRightTicks < -1000000){
      encoderFailure=true;

      deltaRightTicks = imax-deltaRightTicks;
    }
    if (encoderFailure)
    {
      deltaRightTicks=0;
      deltaLeftTicks=0;
      encoderFailure=false;
    }else{
      oldLticks =newLticks;
      oldRticks =newRticks; 
    }
         

  };


  int newLticks=0;
  int newRticks=0;
  int oldLticks=0;
  int oldRticks=0;
  int deltaLeftTicks=0;
  int deltaRightTicks=0;

  bool isFirst=true;

  rclcpp::Time oldTime;
  rclcpp::Time newTime;

  rclcpp::Subscription<traxter_msgs::msg::DualMotorArray>::SharedPtr ticks_subscription_;
  rclcpp::Subscription<traxter_msgs::msg::DualMotorArray>::SharedPtr command_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr wheel_speed_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr wheel_command_publisher_;
  geometry_msgs::msg::Point outmsg;
  const double PI = 3.1415926535;
  const int TICKSPERWHEELREV=360;
  u_int8_t status;
  const int imin = std::numeric_limits<int>::min(); // minimum value (-2147483647)
  const int imax = std::numeric_limits<int>::max(); // maximum value (2147483647)

  };

  int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Create a loop that will keep the program in execution
  rclcpp::spin(std::make_shared<WheelSpeedHelper>());
  rclcpp::shutdown();
  return 0;
}
