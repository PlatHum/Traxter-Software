

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>
#include <traxter_msgs/msg/dual_motor_array.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
using std::placeholders::_1;

class cmdVelConverter : public rclcpp::Node
{
public:
  // Initiate a Node called 'simple_subscriber'
  cmdVelConverter()
  : Node("cmd_vel_interpreter")
  {

    this->declare_parameter("_wheel_radius", 0.0355);
    this->declare_parameter("_wheel_base", 0.225);
    this->declare_parameter("_run_type", 2);//0 full simulation, 1 hardware in the loop, 2 real hardware

    // Create a Subscriber object that will listen to the /counter topic and will call the 'topic_callback' function       // each time it reads something from the topic
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 2,std::bind(&cmdVelConverter::topic_callback, this, _1));

    motor_command.status=1;
    this->get_parameter("_wheel_radius", WHEEL_RADIUS);
    this->get_parameter("_wheel_base", WHEEL_BASE);
    this->get_parameter("_run_type", RUN_TYPE);

    switch (RUN_TYPE)
    {

    case 0: // full simulation

      left_simulation_publisher_ = this->create_publisher<std_msgs::msg::Float64>("traxter/simulation/command/left",1);
      right_simulation_publisher_ = this->create_publisher<std_msgs::msg::Float64>("traxter/simulation/command/right",1);
      break;

    case 1: // hardware in the loop

      hardware_publisher_ = this->create_publisher<traxter_msgs::msg::DualMotorArray>("traxter/motor/command",1);
      left_simulation_publisher_ = this->create_publisher<std_msgs::msg::Float64>("traxter/simulation/command/left",1);
      right_simulation_publisher_ = this->create_publisher<std_msgs::msg::Float64>("traxter/simulation/command/right",1);
      break;

    case 2: //real hardware

      hardware_publisher_ = this->create_publisher<traxter_msgs::msg::DualMotorArray>("traxter/motor/command",1);
      break;

    default:
      RCLCPP_ERROR(this->get_logger(), "Could not interpret run type. Assuming full simulation.");
      RUN_TYPE=0;
      left_simulation_publisher_ = this->create_publisher<std_msgs::msg::Float64>("traxter/simulation/command/left",1);
      right_simulation_publisher_ = this->create_publisher<std_msgs::msg::Float64>("traxter/simulation/command/right",1);
      break;
    }

  }

private:
  // Define a function called 'topic_callback' that receives a parameter named 'msg' 
  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {

    switch (RUN_TYPE)
    {
    case 1: // hardware in the loop


      left_command.data=(msg->linear.x - msg->angular.z * WHEEL_BASE/2)/WHEEL_RADIUS;
      right_command.data=(msg->linear.x + msg->angular.z * WHEEL_BASE/2)/WHEEL_RADIUS;
      motor_command.motor1=int(left_command.data*100); //x100
      motor_command.motor2=int(right_command.data*100); //x100
      hardware_publisher_->publish(motor_command);
      left_simulation_publisher_ ->publish(left_command);
      right_simulation_publisher_->publish(right_command);
      break;

    case 2: //real hardware

      motor_command.motor1=int((msg->linear.x - msg->angular.z * WHEEL_BASE/2)*100/WHEEL_RADIUS); //x100
      motor_command.motor2=int((msg->linear.x + msg->angular.z * WHEEL_BASE/2)*100/WHEEL_RADIUS); //x100
      hardware_publisher_->publish(motor_command);
      break;
    
    default: // full simulation

      left_command.data=(msg->linear.x - msg->angular.z * WHEEL_BASE/2)/WHEEL_RADIUS;
      right_command.data=(msg->linear.x + msg->angular.z * WHEEL_BASE/2)/WHEEL_RADIUS;
      left_simulation_publisher_ ->publish(left_command);
      right_simulation_publisher_->publish(right_command);
      break;
    }
  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<traxter_msgs::msg::DualMotorArray>::SharedPtr hardware_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_simulation_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_simulation_publisher_;

  traxter_msgs::msg::DualMotorArray motor_command;
  std_msgs::msg::Float64 left_command;
  std_msgs::msg::Float64 right_command;

//parameter holders
double WHEEL_RADIUS;
double WHEEL_BASE; //223.8375mm actually
int RUN_TYPE;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Create a loop that will keep the program in execution
  rclcpp::spin(std::make_shared<cmdVelConverter>());
  rclcpp::shutdown();
  return 0;
}

