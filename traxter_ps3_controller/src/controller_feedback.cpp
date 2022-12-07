

#include "rclcpp/rclcpp.hpp"
#include <chrono>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/joy_feedback.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;


class ControllerFeedbackNode : public rclcpp::Node
{
public:
  // Initiate a Node called 'simple_subscriber'
  ControllerFeedbackNode()
  : Node("controller_feedback")
  {

    this->declare_parameter("_controller_feedback_distance", 0.2);
    this->declare_parameter("_controller_feedback_intensity", 0.3);
    this->get_parameter("_controller_feedback_distance", MIN_DISTANCE);
    this->get_parameter("_controller_feedback_intensity", INTENSITY);
    if(INTENSITY>1.0){INTENSITY=1.0;}
    if(INTENSITY<0.0){INTENSITY=0.0;}
    if(MIN_DISTANCE<0.20){MIN_DISTANCE=0.20;}

    RCLCPP_INFO(this->get_logger(), "Controller Feedback on %.1fm with %.1f intensity rumble.",MIN_DISTANCE,INTENSITY );
    
    publisher_ = this->create_publisher<sensor_msgs::msg::JoyFeedback>("joy/set_feedback", 1);
    timer_ = this->create_wall_timer(80ms, std::bind(&ControllerFeedbackNode::timer_callback, this));

    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
          "scan", 2, std::bind(&ControllerFeedbackNode::scan_topic_callback, this, _1));

    msg_out.id=0;
    msg_out.type=1;


    }


private:

void scan_topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){

    for (size_t i = 0; i < msg->ranges.size(); i++)
    {
        if (msg->ranges[i]<=MIN_DISTANCE && msg->ranges[i]<=msg->range_max && msg->ranges[i]>=msg->range_min){
            shouldRumble=true;
             RCLCPP_INFO(this->get_logger(), "Found point %.2fm away from robot",msg->ranges[i]);
            return;
        }
    }
    shouldRumble=false; 
}

void timer_callback(){

    if (shouldRumble){
        msg_out.intensity=INTENSITY;
    }else{
        msg_out.intensity=0.0;
    }
    publisher_->publish(msg_out);

};


  

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;

  rclcpp::Publisher<sensor_msgs::msg::JoyFeedback>::SharedPtr publisher_;
  bool shouldRumble=false;
  float MIN_DISTANCE;
  float INTENSITY;
  sensor_msgs::msg::JoyFeedback msg_out;

  rclcpp::TimerBase::SharedPtr timer_;
  };



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Create a loop that will keep the program in execution
  rclcpp::spin(std::make_shared<ControllerFeedbackNode>());
  rclcpp::shutdown();
  return 0;
}

