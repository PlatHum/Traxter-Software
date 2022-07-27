#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>
#include <traxter_msgs/msg/dual_motor_array.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <math.h>
#include <limits.h>
using std::placeholders::_1;

class odometryPublisher : public rclcpp::Node
{
public:
  // Initiate a Node called 'simple_subscriber'
  odometryPublisher()
  : Node("odometry_publisher")
  {

    this->declare_parameter("_run_type", 0);//0 full simulation, 1 hardware in the loop, 2 real hardware
    this->declare_parameter("_odom_type", 0);//0 classic euler, 1 classic trapezoidal, 2 classic exact, 3 geometric trapezoidal
    this->declare_parameter("_wheel_radius", 0.0355);
    this->declare_parameter("_wheel_base", 0.225);
    this->declare_parameter("_ticks_per_meter", 1613);//assume nominal if not especified
    this->declare_parameter("_ticks_per_wheel_rev", 360);
    this->declare_parameter("_initial_x", 0.0);
    this->declare_parameter("_initial_y", 0.0);
    this->declare_parameter("_initial_theta", 0.00000000001);
    this->declare_parameter("_odometry_covariance", std::vector<double>{.01, .01, .01, .01, .01, .01,.165,.165,.165,.165,.165,.165});

    rclcpp::QoS qos(3);
    qos.keep_last(3);
    qos.best_effort();
    qos.durability_volatile();

    this->get_parameter("_wheel_radius", WHEEL_RADIUS);
    this->get_parameter("_wheel_base", WHEEL_BASE);
    this->get_parameter("_run_type", RUN_TYPE);
    this->get_parameter("_odom_type", ODOM_TYPE);
    this->get_parameter("_ticks_per_meter", TICKSPERMETER);
    this->get_parameter("_ticks_per_wheel_rev", TICKSPERWHEELREV);
    this->get_parameter("_initial_x", INITIAL_X);
    this->get_parameter("_initial_y", INITIAL_Y);
    this->get_parameter("_initial_theta", INITIAL_THETA);
    this->get_parameter("_odometry_covariance", ODOMETRY_COVARIANCE);

    switch (_runType)
    {
    case 0:
        RCLCPP_INFO(this->get_logger(), "Odometry running in full Simulation mode.");
        simul_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
          "traxter/simulation/joint_states", std::bind(&odometryPublisher::simulation_topic_callback, this, _1));
      break;
    case 1:
        RCLCPP_INFO(this->get_logger(), "Odometry running in Hardware in the Loop mode.");
        inLoop_subscription_ = this->create_subscription<traxter_msgs::msg::DualMotorArray>(
          "traxter/encoder/ticks", qos, std::bind(&odometryPublisher::hardware_topic_callback, this, _1));
      break;
    case 2:
        RCLCPP_INFO(this->get_logger(), "Odometry running in Hardware mode.");
        hardware_subscription_ = this->create_subscription<traxter_msgs::msg::DualMotorArray>(
          "traxter/encoder/ticks", qos, std::bind(&odometryPublisher::hardware_topic_callback, this, _1));
      break;

    default:
        RCLCPP_ERROR(this->get_logger(), "Could not interpret run type. Assuming full simulation.");
        RUN_TYPE=0;
      break;
    }

    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("traxter/odometry/raw",1);
    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",1);

    setup_variables();
  }

private:

  void setup_variables()
  {
    //linear
    newOdom.pose.covariance[0] = ODOMETRY_COVARIANCE[0];
    oldOdom.pose.covariance[0] = ODOMETRY_COVARIANCE[0];
    newOdom.pose.covariance[7] = ODOMETRY_COVARIANCE[1];
    oldOdom.pose.covariance[7] = ODOMETRY_COVARIANCE[1];
    newOdom.pose.covariance[14] = ODOMETRY_COVARIANCE[2];
    oldOdom.pose.covariance[14] = ODOMETRY_COVARIANCE[2];

    newOdom.twist.covariance[0] = ODOMETRY_COVARIANCE[3];
    oldOdom.twist.covariance[0] = ODOMETRY_COVARIANCE[3];
    newOdom.twist.covariance[7] = ODOMETRY_COVARIANCE[4];
    oldOdom.twist.covariance[7] = ODOMETRY_COVARIANCE[4];
    newOdom.twist.covariance[14] = ODOMETRY_COVARIANCE[5];
    oldOdom.twist.covariance[14] = ODOMETRY_COVARIANCE[5];

    //angular
    newOdom.pose.covariance[21] = ODOMETRY_COVARIANCE[6];
    oldOdom.pose.covariance[21] = ODOMETRY_COVARIANCE[6];
    newOdom.pose.covariance[28] = ODOMETRY_COVARIANCE[7];
    oldOdom.pose.covariance[28] = ODOMETRY_COVARIANCE[7];
    newOdom.pose.covariance[35] = ODOMETRY_COVARIANCE[8];
    oldOdom.pose.covariance[35] = ODOMETRY_COVARIANCE[8];

    newOdom.pose.covariance[21] = ODOMETRY_COVARIANCE[9];
    oldOdom.pose.covariance[21] = ODOMETRY_COVARIANCE[9];
    newOdom.pose.covariance[28] = ODOMETRY_COVARIANCE[10];
    oldOdom.pose.covariance[28] = ODOMETRY_COVARIANCE[10];
    newOdom.pose.covariance[35] = ODOMETRY_COVARIANCE[11];
    oldOdom.pose.covariance[35] = ODOMETRY_COVARIANCE[11];

    for(int i = 0; i<36; i++)
    {
       if(!(i == 0 || i == 7 || i == 14 || i == 21 || i == 28 || i== 35))
       {
          newOdom.pose.covariance[i] = 0;
          oldOdom.pose.covariance[i] = 0;

          newOdom.twist.covariance[i] = 0; 
          oldOdom.twist.covariance[i] = 0;
       }
    }

    oldOdom.pose.pose.position.x = INITIAL_X;
    oldOdom.pose.pose.position.y = INITIAL_Y;
    oldOdom.pose.pose.position.z = 0;
    tf2::Quaternion tempQuaternion;
    tempQuaternion.setRPY( 0, 0, INITIAL_THETA );
    oldOdom.pose.pose.orientation.x = tempQuaternion.x();
    oldOdom.pose.pose.orientation.y = tempQuaternion.y();
    oldOdom.pose.pose.orientation.z = tempQuaternion.z();
    oldOdom.pose.pose.orientation.w = tempQuaternion.w();
    newOdom.header.frame_id = "odom";
    newOdom.child_frame_id = "base_link";
    oldOdom.header.frame_id = "odom";
    oldOdom.child_frame_id = "base_link";    

    oldOdom.header.stamp = rclcpp::Clock().now();  

  }

   
  void hardware_topic_callback(const traxter_msgs::msg::DualMotorArray::SharedPtr msg)
  {
    newLticks= msg->motor1;
    newRticks= msg->motor2;
    status= msg->status;
    if(status==0){ //means de MD25 reset the encoders
      oldRticks=0;
      oldLticks=0;
    }

    deltaEncoderTicks();
    odometryAlgorithm();
  }

  void simulation_topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->name.size(); i++){
      
      if (msg->name[i]=="front_left_wheel_joint"){

        newLticks=int(msg->position[i]/(2*PI)*TICKSPERWHEELREV);

      }else if(msg->name[i]=="front_right_wheel_joint"){

        newRticks=int(msg->position[i]/(2*PI)*TICKSPERWHEELREV);
      }

    }

    deltaEncoderTicks();
    odometryAlgorithm();
  }

  void deltaEncoderTicks(){

    deltaLeftTicks = (newLticks- oldLticks);
    deltaRightTicks = (newRticks- oldRticks);

    if (deltaLeftTicks > 1000000){

      deltaLeftTicks=imin + deltaLeftTicks;
    
    }else if (deltaLeftTicks < -1000000){

      deltaLeftTicks = imax-deltaLeftTicks;
    }

    if (deltaRightTicks > 1000000){

      deltaRightTicks=imin + deltaRightTicks;
    
    }else if (deltaRightTicks < -1000000){

      deltaRightTicks = imax-deltaRightTicks;
    }

    oldLticks =newLticks;
    oldRticks =newRticks;      

  }

  void odometryAlgorithm(){

    switch (ODOM_TYPE)
    {
    case 3 //geometric approach
      experimentalOdom()
      break;
    
    default:
      break;
    }

    publisher_->publish(newOdom);
    oldYaw=newYaw;
    oldOdom=newOdom;

  }

  double reframeAngle(double angle){

    if (angle > PI)
        {
        angle -= 2*PI;
        }
    else if (angle < -PI)
    {
        angle += 2*PI;
    }
    return angle;

  }

  void experimentalOdom()
  {   

    double leftDistance = deltaLeftTicks/TICKSPERMETER;
    double rightDistance = deltaRightTicks/TICKSPERMETER;

    //average distance
    double cycleDistance = (rightDistance+leftDistance)/2;
    //how many radians robot has turned since last cycle
    double cycleAngle = asin((rightDistance-leftDistance)/WHEEL_BASE);

    //average angle during last cycle (for trapezoidal numerical integration)
    double avgAngle = cycleAngle/2 + oldYaw;

    avgAngle=reframeAngle(avgAngle);

    //calculate new x, y, and theta
    newOdom.pose.pose.position.x = oldOdom.pose.pose.position.x + cos(avgAngle)*cycleDistance;
    newOdom.pose.pose.position.y = oldOdom.pose.pose.position.y + sin(avgAngle)*cycleDistance;
    newYaw = cycleAngle + oldYaw;

    //prevent lockup from a single erroneous cycle
    if(isnan(newOdom.pose.pose.position.x) || isnan(newOdom.pose.pose.position.y) || isnan(newYaw) )
    {
        newOdom.pose.pose.position.x = oldOdom.pose.pose.position.x;
        newOdom.pose.pose.position.y = oldOdom.pose.pose.position.y;
        newYaw=oldYaw;
        newOdom.twist.twist.linear.x = oldOdom.twist.twist.linear.x;
        newOdom.twist.twist.linear.y = oldOdom.twist.twist.linear.y;
    }else{

      //keep theta in range proper range
      newYaw=reframeAngle(newYaw);

      tf2::Quaternion q;
      q.setRPY(0, 0, newYaw);

      newOdom.pose.pose.orientation.x = q.x();
      newOdom.pose.pose.orientation.y = q.y();
      newOdom.pose.pose.orientation.z = q.z();
      newOdom.pose.pose.orientation.w = q.w();

      //calculate velocity
      newOdom.header.stamp = rclcpp::Clock().now();
      newOdom.twist.twist.linear.x = cycleDistance/(newOdom.header.stamp.sec- oldOdom.header.stamp.sec);
      newOdom.twist.twist.angular.z = cycleAngle/(newOdom.header.stamp.sec - oldOdom.header.stamp.sec);
    }
      //save odom x, y, and theta for use in next cycle
/*       oldOdom.pose.pose.position.x = newOdom.pose.pose.position.x;
      oldOdom.pose.pose.position.y = newOdom.pose.pose.position.y;
      oldYaw=newYaw;
      oldOdom.header.stamp = newOdom.header.stamp; */

  }

  void classicEulerOdom(){

  }


  rclcpp::Subscription<traxter_msgs::msg::DualMotorArray>::SharedPtr hardware_subscription_;
  rclcpp::Subscription<traxter_msgs::msg::DualMotorArray>::SharedPtr inLoop_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr simul_subscription_;
  nav_msgs::msg::Odometry newOdom;
  nav_msgs::msg::Odometry oldOdom;
  const double PI = 3.141592;

  double oldYaw;
  double newYaw;

  int _runType=0;


  int newLticks=0;
  int newRticks=0;
  int oldLticks=0;
  int oldRticks=0;
  int deltaLeftTicks=0;
  int deltaRightTicks=0;


  u_int8_t status;
  const int imin = std::numeric_limits<int>::min(); // minimum value (-2147483647)
  const int imax = std::numeric_limits<int>::max(); // maximum value (2147483647)

//parameter holders
  int TICKSPERWHEELREV; //not in use. just a reference for now
  int TICKSPERMETER;
  double WHEEL_RADIUS;
  double WHEEL_BASE; //223.8375mm actually
  int RUN_TYPE;
  int ODOM_TYPE;
  double INITIAL_X;
  double INITIAL_Y;
  double INITIAL_THETA;
  std::vector<double> ODOMETRY_COVARIANCE;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Create a loop that will keep the program in execution
  rclcpp::spin(std::make_shared<odometryPublisher>());
  rclcpp::shutdown();
  return 0;
}
