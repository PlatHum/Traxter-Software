#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>
#include <traxter_msgs/msg/dual_motor_array.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <math.h>
#include <limits.h>
using std::placeholders::_1;

class odometryPublisher : public rclcpp::Node
{
public:

  odometryPublisher()
  : Node("odometry_publisher")
  {

    this->declare_parameter("_run_type", 2);//0 full simulation, 1 hardware in the loop, 2 real hardware
    this->declare_parameter("_odom_type", 1);//0 classic euler, 1 classic trapezoidal, 2 classic exact, 3 experimental, 4 debug
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

    switch (RUN_TYPE)
    {
    case runTypeList::fullSimul:
        RCLCPP_INFO(this->get_logger(), "Odometry running in full Simulation mode.");
        simul_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
          "joint_states", 2, std::bind(&odometryPublisher::simulation_topic_callback, this, _1));
      break;
    case runTypeList::inLoop: //hardware in the loop
        RCLCPP_INFO(this->get_logger(), "Odometry running in Hardware in the Loop mode.");
        inLoop_subscription_ = this->create_subscription<traxter_msgs::msg::DualMotorArray>(
          "traxter/encoder/ticks", qos, std::bind(&odometryPublisher::hardware_topic_callback, this, _1));
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",1);
      break;
    case runTypeList::realHard: //hardware
        RCLCPP_INFO(this->get_logger(), "Odometry running in Hardware mode.");
        hardware_subscription_ = this->create_subscription<traxter_msgs::msg::DualMotorArray>(
          "traxter/encoder/ticks", qos, std::bind(&odometryPublisher::hardware_topic_callback, this, _1));
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",1);
      break;
    default:
        RCLCPP_FATAL(this->get_logger(), "Could not interpret run type. Could not construct odometry node.");
      break;
    }

  if (ODOM_TYPE==odomTypeList::debug){
    classicEuler_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("traxter/odometry/raw/classicEuler",1);
    classicEuler_odom_path_publisher_=this->create_publisher<nav_msgs::msg::Path>("traxter/path/odometry/raw/classicEuler",1);
    classicTrapezoidal_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("traxter/odometry/raw/classicTrapezoidal",1);
    classicTrapezoidal_odom_path_publisher_=this->create_publisher<nav_msgs::msg::Path>("traxter/path/odometry/raw/classicTrapezoidal",1);
    classicExact_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("traxter/odometry/raw/classicExact",1);
    classicExact_odom_path_publisher_=this->create_publisher<nav_msgs::msg::Path>("traxter/path/odometry/raw/classicExact",1);
    experimental_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("traxter/odometry/raw/experimental",1);
    experimental_odom_path_publisher_=this->create_publisher<nav_msgs::msg::Path>("traxter/path/odometry/raw/experimental",1);
  }else{
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("traxter/odometry/raw",1);
    odom_path_publisher_=this->create_publisher<nav_msgs::msg::Path>("traxter/path/odometry/raw",1);
  }

    setup_variables();
  };

private:

  void setup_variables()
  {
    //linear
    newOdom.pose.covariance[0] = ODOMETRY_COVARIANCE[0];
    
    newOdom.pose.covariance[7] = ODOMETRY_COVARIANCE[1];
    
    newOdom.pose.covariance[14] = ODOMETRY_COVARIANCE[2];
    

    newOdom.twist.covariance[0] = ODOMETRY_COVARIANCE[3];
    
    newOdom.twist.covariance[7] = ODOMETRY_COVARIANCE[4];
    
    newOdom.twist.covariance[14] = ODOMETRY_COVARIANCE[5];
    

    //angular
    newOdom.pose.covariance[21] = ODOMETRY_COVARIANCE[6];
    
    newOdom.pose.covariance[28] = ODOMETRY_COVARIANCE[7];
    
    newOdom.pose.covariance[35] = ODOMETRY_COVARIANCE[8];
    

    newOdom.pose.covariance[21] = ODOMETRY_COVARIANCE[9];
    
    newOdom.pose.covariance[28] = ODOMETRY_COVARIANCE[10];
    
    newOdom.pose.covariance[35] = ODOMETRY_COVARIANCE[11];
    

    for(int i = 0; i<36; i++)
    {
       if(!(i == 0 || i == 7 || i == 14 || i == 21 || i == 28 || i== 35))
       {
          newOdom.pose.covariance[i] = 0;
          
          newOdom.twist.covariance[i] = 0; 
          
       }
    }

    oldXClassicEuler = INITIAL_X;
    oldXClassicTrapezoidal = INITIAL_X;
    oldXClassicExact = INITIAL_X;
    oldXExperimental = INITIAL_X;

    oldYClassicEuler = INITIAL_Y;
    oldYClassicTrapezoidal = INITIAL_Y;
    oldYClassicExact = INITIAL_Y;
    oldYExperimental = INITIAL_Y;
    
    oldYawClassicEuler = INITIAL_THETA;
    oldYawClassicTrapezoidal = INITIAL_THETA;
    oldYawClassicExact = INITIAL_THETA;
    oldYawExperimental = INITIAL_THETA;
    tf2::Quaternion q;
    q.setRPY(0, 0, INITIAL_THETA);

    newOdom.pose.pose.orientation.x = q.x();
    newOdom.pose.pose.orientation.y = q.y();
    newOdom.pose.pose.orientation.z = q.z();
    newOdom.pose.pose.orientation.w = q.w();

    newOdom.pose.pose.position.x = INITIAL_X;
    newOdom.pose.pose.position.y = INITIAL_Y;

    newOdom.header.frame_id = "odom";
    newOdom.child_frame_id = "base_link";

    traxter_joints.name = {"front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint", "rear_right_wheel_joint"};   
    
    oldTime=this->now();
    newOdom.header.stamp = this->now();

    switch (ODOM_TYPE){
    case odomTypeList::classicEuler:
      RCLCPP_INFO(this->get_logger(), "Selected classic euler odometry calculation.");
      updateOdomPath(odomPathClassicEuler);
      break;
    case odomTypeList::classicTrapezoidal:
      RCLCPP_INFO(this->get_logger(), "Selected classic trapezoidal odometry calculation.");
      updateOdomPath(odomPathClassicTrapezoidal);
      break;
    case odomTypeList::classicExact:
      RCLCPP_INFO(this->get_logger(), "Selected classic exact odometry calculation.");
      updateOdomPath(odomPathClassicExact);
      break;
    case odomTypeList::experimental:
      RCLCPP_INFO(this->get_logger(), "Selected experimental odometry calculation.");
      updateOdomPath(odomPathExperimental);
      break;
    case odomTypeList::debug:
      RCLCPP_INFO(this->get_logger(), "Selected debug mode odometry calculation.");
      updateOdomPath(odomPathClassicEuler);
      updateOdomPath(odomPathClassicTrapezoidal);
      updateOdomPath(odomPathClassicExact);
      updateOdomPath(odomPathExperimental);
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Could not interpret odometry calculation. Assuming classic trapezoidal.");
      ODOM_TYPE=odomTypeList::classicTrapezoidal;
      updateOdomPath(odomPathClassicTrapezoidal);
      break;
    }
  };

   
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
    publishJointState();
  };

  void simulation_topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    int wanted_joint_counter=0;
    for (size_t i = 0; i < msg->name.size(); i++){
      
      if (msg->name[i]=="front_left_wheel_joint"){

        newLticks=std::floor(msg->position[i]/(2*PI)*TICKSPERWHEELREV);
        wanted_joint_counter++;

      }else if(msg->name[i]=="front_right_wheel_joint"){

        newRticks=std::floor(msg->position[i]/(2*PI)*TICKSPERWHEELREV);
        wanted_joint_counter++;
      }
      if (wanted_joint_counter==2){
        break;
      }
    }

    deltaEncoderTicks();
    odometryAlgorithm();
  };

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

  };

  void odometryAlgorithm(){

    newTime=this->now();
    newOdom.header.stamp = this->now();

    switch (ODOM_TYPE){
    case odomTypeList::classicEuler:
      estimateVelocityUsingTicks();
      classicEulerOdom();
      odom_publisher_->publish(newOdom);
      updateOdomPath(odomPathClassicEuler);
      odom_path_publisher_->publish(odomPathClassicEuler);
      break;
    case odomTypeList::classicTrapezoidal:
      estimateVelocityUsingTicks();
      classicTrapezoidalOdom();
      odom_publisher_->publish(newOdom);
      updateOdomPath(odomPathClassicTrapezoidal);
      odom_path_publisher_->publish(odomPathClassicTrapezoidal);
      break;
    case odomTypeList::classicExact:
      estimateVelocityUsingTicks();
      classicExactOdom();
      odom_publisher_->publish(newOdom);
      updateOdomPath(odomPathClassicExact);
      odom_path_publisher_->publish(odomPathClassicExact);
      break;
    case odomTypeList::experimental:
      experimentalOdom();
      odom_publisher_->publish(newOdom);
      updateOdomPath(odomPathExperimental);
      odom_path_publisher_->publish(odomPathExperimental);
      break;
    case odomTypeList::debug:
      estimateVelocityUsingTicks();
      classicEulerOdom();
      classicEuler_odom_publisher_->publish(newOdom);
      updateOdomPath(odomPathClassicEuler);
      classicEuler_odom_path_publisher_->publish(odomPathClassicEuler);

      classicTrapezoidalOdom();
      classicTrapezoidal_odom_publisher_->publish(newOdom);
      updateOdomPath(odomPathClassicTrapezoidal);
      classicTrapezoidal_odom_path_publisher_->publish(odomPathClassicTrapezoidal);

      classicExactOdom();
      classicExact_odom_publisher_->publish(newOdom);
      updateOdomPath(odomPathClassicExact);
      classicExact_odom_path_publisher_->publish(odomPathClassicExact);

      experimentalOdom();
      experimental_odom_publisher_->publish(newOdom);
      updateOdomPath(odomPathExperimental);
      experimental_odom_path_publisher_->publish(odomPathExperimental);
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Could not interpret odom type. Assuming classic trapezoidal.");
      ODOM_TYPE=odomTypeList::classicTrapezoidal;
      estimateVelocityUsingTicks();
      classicTrapezoidalOdom();
      odom_publisher_->publish(newOdom);
      updateOdomPath(odomPathClassicTrapezoidal);
      odom_path_publisher_->publish(odomPathClassicTrapezoidal);
      break;
    }

  };

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

  };

 bool checkErroneousCycle(double previous_x, double previous_y, double yaw){

    //prevent lockup from a single erroneous cycle
    if(isnan(newOdom.pose.pose.position.x) || isnan(newOdom.pose.pose.position.y) || isnan(yaw) )
    {
        newOdom.pose.pose.position.x = previous_x;
        newOdom.pose.pose.position.y = previous_y;
        return true;
    }else{

        return false;
    }

 }

  void estimateVelocityUsingTicks(){

    double deltaT=(newTime - oldTime).nanoseconds()*1e-9;

    double leftWheelSpeed=deltaLeftTicks/deltaT *2*PI / TICKSPERWHEELREV;
    double rightWheelSpeed=deltaRightTicks/deltaT *2*PI / TICKSPERWHEELREV;

    newOdom.twist.twist.linear.x = (leftWheelSpeed+rightWheelSpeed)*WHEEL_RADIUS/2;
    newOdom.twist.twist.linear.y = 0;
    newOdom.twist.twist.angular.z = (rightWheelSpeed-leftWheelSpeed)*WHEEL_RADIUS/WHEEL_BASE;

  };

  void classicEulerOdom(){

    double deltaT=(newTime - oldTime).nanoseconds()*1e-9;

    double newYaw= newOdom.twist.twist.angular.z*deltaT + oldYawClassicEuler;

    //keep theta in range proper range
    newYaw=reframeAngle(newYaw);

    newOdom.pose.pose.position.x=cos(newYaw)*newOdom.twist.twist.linear.x*deltaT+oldXClassicEuler;
    newOdom.pose.pose.position.y=sin(newYaw)*newOdom.twist.twist.linear.x*deltaT+oldYClassicEuler;

    //prevent lockup from a single erroneous cycle
    if(!checkErroneousCycle(oldXClassicEuler,oldYClassicEuler, newYaw)){
      tf2::Quaternion q;
      q.setRPY(0, 0, newYaw);

      newOdom.pose.pose.orientation.x = q.x();
      newOdom.pose.pose.orientation.y = q.y();
      newOdom.pose.pose.orientation.z = q.z();
      newOdom.pose.pose.orientation.w = q.w();
      oldYawClassicEuler=newYaw;
      oldXClassicEuler=newOdom.pose.pose.position.x;
      oldYClassicEuler=newOdom.pose.pose.position.y;
    }
  };

  void classicTrapezoidalOdom(){


    double deltaT=(newTime - oldTime).nanoseconds()*1e-9;

    double newYaw= newOdom.twist.twist.angular.z*deltaT + oldYawClassicTrapezoidal;

    //keep theta in range proper range
    newYaw=reframeAngle(newYaw);
    double trapezoidalYaw= newOdom.twist.twist.angular.z*deltaT/2 + oldYawClassicTrapezoidal;

    newOdom.pose.pose.position.x=cos(trapezoidalYaw)*newOdom.twist.twist.linear.x*deltaT+oldXClassicTrapezoidal;
    newOdom.pose.pose.position.y=sin(trapezoidalYaw)*newOdom.twist.twist.linear.x*deltaT+oldYClassicTrapezoidal;

    //prevent lockup from a single erroneous cycle
    if(!checkErroneousCycle(oldXClassicTrapezoidal,oldYClassicTrapezoidal, newYaw)){
      tf2::Quaternion q;
      q.setRPY(0, 0, newYaw);

      newOdom.pose.pose.orientation.x = q.x();
      newOdom.pose.pose.orientation.y = q.y();
      newOdom.pose.pose.orientation.z = q.z();
      newOdom.pose.pose.orientation.w = q.w();
      oldYawClassicTrapezoidal=newYaw;
      oldXClassicTrapezoidal=newOdom.pose.pose.position.x;
      oldYClassicTrapezoidal=newOdom.pose.pose.position.y;
    }
  };

  void classicExactOdom(){

    double deltaT=(newTime - oldTime).nanoseconds()*1e-9;
    double newYaw= newOdom.twist.twist.angular.z*deltaT + oldYawClassicExact;
    //keep theta in range proper range
    newYaw=reframeAngle(newYaw);

    if (abs(newOdom.twist.twist.angular.z)>0.01){
      double ratio=newOdom.twist.twist.linear.x/newOdom.twist.twist.angular.z;
      newOdom.pose.pose.position.x= oldXClassicExact + ratio*(sin(newYaw)-sin(oldYawClassicExact));
      newOdom.pose.pose.position.y= oldYClassicExact - ratio*(cos(newYaw)-cos(oldYawClassicExact));
    }else{
      newOdom.pose.pose.position.x=cos(newYaw)*newOdom.twist.twist.linear.x*deltaT+oldXClassicExact;
      newOdom.pose.pose.position.y=sin(newYaw)*newOdom.twist.twist.linear.x*deltaT+oldYClassicExact;
    }

    //prevent lockup from a single erroneous cycle
    if(!checkErroneousCycle(oldXClassicExact,oldYClassicExact, newYaw)){
      tf2::Quaternion q;
      q.setRPY(0, 0, newYaw);

      newOdom.pose.pose.orientation.x = q.x();
      newOdom.pose.pose.orientation.y = q.y();
      newOdom.pose.pose.orientation.z = q.z();
      newOdom.pose.pose.orientation.w = q.w();
      oldYawClassicExact=newYaw;
      oldXClassicExact=newOdom.pose.pose.position.x;
      oldYClassicExact=newOdom.pose.pose.position.y;
    }
  };

  void experimentalOdom(){   

    double leftDistance = static_cast<float>(deltaLeftTicks)/TICKSPERMETER;
    double rightDistance = static_cast<float>(deltaRightTicks)/TICKSPERMETER;

    //average distance
    double cycleDistance = (rightDistance+leftDistance)/2;

    //how many radians robot has turned since last cycle
    double cycleAngle = asin ((rightDistance-leftDistance)/WHEEL_BASE);

    //average angle during last cycle (for trapezoidal numerical integration)
    double avgAngle = cycleAngle/2 + oldYawExperimental;

    avgAngle=reframeAngle(avgAngle);

    //calculate new x, y, and theta
    newOdom.pose.pose.position.x = oldXExperimental + cos(avgAngle)*cycleDistance;
    newOdom.pose.pose.position.y = oldYExperimental + sin(avgAngle)*cycleDistance;
    double newYaw = cycleAngle + oldYawExperimental;

    //calculate velocity
    double deltaT=(newTime - oldTime).nanoseconds()*1e-9;
    newOdom.twist.twist.linear.x = cycleDistance/deltaT;
    newOdom.twist.twist.angular.z = cycleAngle/deltaT;
    newYaw=reframeAngle(newYaw);    

    //prevent lockup from a single erroneous cycle
    if(!checkErroneousCycle(oldXExperimental,oldYExperimental, newYaw)){
      tf2::Quaternion q;
      q.setRPY(0, 0, newYaw);

      newOdom.pose.pose.orientation.x = q.x();
      newOdom.pose.pose.orientation.y = q.y();
      newOdom.pose.pose.orientation.z = q.z();
      newOdom.pose.pose.orientation.w = q.w();
      oldYawExperimental=newYaw;
      oldXExperimental=newOdom.pose.pose.position.x;
      oldYExperimental=newOdom.pose.pose.position.y;
      return;
    }
  };


  void publishJointState(){
    double left_side_position=newLticks/360*PI/180;
    double right_side_position=newRticks/360*PI/180;

    traxter_joints.position.clear();
    traxter_joints.position={left_side_position,right_side_position,left_side_position,right_side_position};
    traxter_joints.header.stamp=this->now();
    joint_state_publisher_->publish(traxter_joints); 

  };

 void updateOdomPath(nav_msgs::msg::Path& pathToUpdate){

    geometry_msgs::msg::PoseStamped tempPose;
    tempPose.header=newOdom.header;
    tempPose.pose.position.x = newOdom.pose.pose.position.x;
    tempPose.pose.position.y = newOdom.pose.pose.position.y;
    pathToUpdate.header=newOdom.header;
    pathToUpdate.poses.push_back(tempPose);
  };


  rclcpp::Subscription<traxter_msgs::msg::DualMotorArray>::SharedPtr hardware_subscription_;
  rclcpp::Subscription<traxter_msgs::msg::DualMotorArray>::SharedPtr inLoop_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odom_path_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr simul_subscription_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr classicEuler_odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr classicEuler_odom_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr classicTrapezoidal_odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr classicTrapezoidal_odom_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr classicExact_odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr classicExact_odom_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr experimental_odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr experimental_odom_path_publisher_;

  nav_msgs::msg::Odometry newOdom;
  sensor_msgs::msg::JointState traxter_joints;
  const double PI = 3.141592;

  enum runTypeList {fullSimul,inLoop,realHard};
  enum odomTypeList {classicEuler,classicTrapezoidal, classicExact, experimental, debug};

  nav_msgs::msg::Path odomPathClassicEuler;
  nav_msgs::msg::Path odomPathClassicTrapezoidal;
  nav_msgs::msg::Path odomPathClassicExact;
  nav_msgs::msg::Path odomPathExperimental;

  double oldXClassicEuler;
  double oldXClassicTrapezoidal;
  double oldXClassicExact;
  double oldXExperimental;
  double oldYClassicEuler;
  double oldYClassicTrapezoidal;
  double oldYClassicExact;
  double oldYExperimental;
  double oldYawClassicEuler;
  double oldYawClassicTrapezoidal;
  double oldYawClassicExact;
  double oldYawExperimental;

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
  double WHEEL_BASE;
  int RUN_TYPE;
  int ODOM_TYPE;
  double INITIAL_X;
  double INITIAL_Y;
  double INITIAL_THETA;
  std::vector<double> ODOMETRY_COVARIANCE;

//time instances because rclcpp is a mess with time
rclcpp::Time oldTime;
rclcpp::Time newTime;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Create a loop that will keep the program in execution
  rclcpp::spin(std::make_shared<odometryPublisher>());
  rclcpp::shutdown();
  return 0;
}
