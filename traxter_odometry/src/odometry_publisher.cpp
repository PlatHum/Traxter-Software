#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>
#include <traxter_msgs/msg/dual_motor_array.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <math.h>
#include <cstdio>  // for EOF
#include <string>
#include <sstream>
#include <vector>
#include <limits.h>
using std::placeholders::_1;
using namespace std::chrono_literals;

class odometryPublisher : public rclcpp::Node
{
public:

  odometryPublisher()
  : Node("odometry_publisher")
  {

    this->declare_parameter("_run_type", 2);//0 full simulation, 1 hardware in the loop, 2 real hardware
    this->declare_parameter("_wheel_radius", 0.0355);
    this->declare_parameter("_wheel_base", 0.225);
    this->declare_parameter("_ticks_per_wheel_rev", 360);
    this->declare_parameter("_initial_x", 0.0);
    this->declare_parameter("_initial_y", 0.0);
    this->declare_parameter("_initial_theta", 0.000000001);
    this->declare_parameter("_k_R", 0.1);
    this->declare_parameter("_k_L", 0.1);
    this->declare_parameter("_E_D", 1.0);
    this->declare_parameter("_E_B", 1.0);
    this->declare_parameter("_publish_odom_path", false);
    /* this->declare_parameter("_update_odometry", false); */
    this->declare_parameter("_limit_covariance", "[[0.0017, 0.00259,  0.00507], [0.00259, 0.0041,  0.0078], [0.00507, 0.0078,  0.018]]");
    this->declare_parameter("_dynamic_covariance", false);
    this->declare_parameter("_publish_odom_tf", false);
/*     this->declare_parameter("_experimental_odom", false);
    this->declare_parameter("_dist_ratio", 1.0);
    this->declare_parameter("_yaw_ratio", 1.0); */
    rclcpp::QoS qos(1);
    qos.keep_last(1);
    qos.best_effort();
    qos.durability_volatile();

    this->get_parameter("_wheel_radius", WHEEL_RADIUS);
    this->get_parameter("_wheel_base", WHEEL_BASE);
    this->get_parameter("_run_type", RUN_TYPE);
    this->get_parameter("_ticks_per_wheel_rev", TICKSPERWHEELREV);
    this->get_parameter("_initial_x", INITIAL_X);
    this->get_parameter("_initial_y", INITIAL_Y);
    this->get_parameter("_initial_theta", INITIAL_THETA);
    this->get_parameter("_k_R", K_R);
    this->get_parameter("_k_L", K_L);
    this->get_parameter("_E_D", E_D);
    this->get_parameter("_E_B", E_B);
    this->get_parameter("_limit_covariance", LIMIT_COVARIANCE);
    this->get_parameter("_dynamic_covariance", DYNAMIC_COVARIANCE);
    this->get_parameter("_publish_odom_tf", PUBLISH_TF);
    this->get_parameter("_publish_odom_path", PUBLISH_PATH);
  

    switch (RUN_TYPE)
    {
    case runTypeList::fullSimul:
        RCLCPP_INFO(this->get_logger(), "Odometry running in full Simulation mode.");
        simul_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
          "joint_states", 1, std::bind(&odometryPublisher::simulation_topic_callback, this, _1));
        publish_timer_ = create_wall_timer( 34ms, std::bind(&odometryPublisher::publish_timer_callback, this));  
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
        return;
      break;
    }

    if (PUBLISH_PATH){
      odom_path_publisher_=this->create_publisher<nav_msgs::msg::Path>("traxter/path/odometry/raw",1);
    }

    if(PUBLISH_TF){
        // Initialize the transform broadcaster
      tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom",1);
    }else{
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("traxter/odometry/raw",1);
    }

    setup_variables();
  };

private:

  void setup_variables()
  {
    std::string ERROR;
    limit_covariance = parseVVF(LIMIT_COVARIANCE, ERROR);

    effectiveWheelBase=WHEEL_BASE/E_B;
    newOdom.pose.pose.position.x = INITIAL_X;
    newOdom.pose.pose.position.y = INITIAL_Y;
    oldOdom.pose.pose.position.x = INITIAL_X;
    oldOdom.pose.pose.position.y = INITIAL_Y;
    tf2::Quaternion q;
    q.setRPY(0, 0, INITIAL_THETA);
    q.normalize();

    newOdom.pose.pose.orientation.x = q.x();
    newOdom.pose.pose.orientation.y = q.y();
    newOdom.pose.pose.orientation.z = q.z();
    newOdom.pose.pose.orientation.w = q.w();
    oldOdom.pose.pose.orientation.x = q.x();
    oldOdom.pose.pose.orientation.y = q.y();
    oldOdom.pose.pose.orientation.z = q.z();
    oldOdom.pose.pose.orientation.w = q.w();

    newOdom.header.frame_id = "odom";
    newOdom.child_frame_id = "base_link";

    traxter_joints.name = {"front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint", "rear_right_wheel_joint"};   
    
    if(DYNAMIC_COVARIANCE){
    motion_increment_covar[0][1]=0.0;
    motion_increment_covar[1][0]=0.0;
    pose_jacobian[0][0]=1.0;
    pose_jacobian[0][1]=0.0;
    pose_jacobian[1][0]=0.0;
    pose_jacobian[1][1]=1.0;
    pose_jacobian[2][1]=0.0;
    pose_jacobian[2][0]=0.0;
    pose_jacobian[2][2]=1.0;
    motion_increment_jacobian[2][0]=1/effectiveWheelBase;
    motion_increment_jacobian[2][1]=-1/effectiveWheelBase;
    }else{
      for (size_t i = 0; i < 3; i++)
      {
        for (size_t j = 0; j <= i; j++)
        {
          newOdom.pose.covariance[mapCov[i][j]]= limit_covariance[i][j];
          newOdom.twist.covariance[mapCov[i][j]]=1.05*limit_covariance[i][j];
          if (i!=j){
            newOdom.pose.covariance[mapCov[j][i]]=newOdom.pose.covariance[mapCov[i][j]];
            newOdom.twist.covariance[mapCov[j][i]]= newOdom.twist.covariance[mapCov[i][j]];
          }
        }
      }
      
    }

    oldTime = this->now();
    newOdom.header.stamp = oldTime;
    oldOdom.header.stamp = newOdom.header.stamp;
    updateOdomPath();
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
    if(isFirst){
      oldRticks=newRticks;
      oldLticks=newLticks;
      isFirst=false;
      oldTime = this->now();      
    }

    deltaEncoderTicks();
    newTime = this->now();
    newOdom.header.stamp = newTime;
    publishJointState();
    runAlgorithm();
  };

  void simulation_topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
      int wanted_joint_counter=0;
      for (size_t i = 0; i < msg->name.size(); i++){
        
        if (msg->name[i]=="front_left_wheel_joint"){

          newLticks=std::floor(((msg->position[i]/(2*PI))*static_cast<double>(TICKSPERWHEELREV)));
          wanted_joint_counter++;

        }else if(msg->name[i]=="front_right_wheel_joint"){

          newRticks=std::floor(((msg->position[i]/(2*PI))*static_cast<double>(TICKSPERWHEELREV)));
          wanted_joint_counter++;
        }
        if (wanted_joint_counter==2){
          break;
        }
      }
    
      newOdom.header.stamp = msg->header.stamp;
      newTime = rclcpp::Time(msg->header.stamp.sec , msg->header.stamp.nanosec);
      deltaEncoderTicks();
      runAlgorithm();
  };

  void deltaEncoderTicks(){

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

  void runAlgorithm(){

    odomAlgorithm();

    if (timeToPublish){
      odom_publisher_->publish(newOdom);
      if(PUBLISH_TF){
         publish_transform();
      }
    }
    if (PUBLISH_PATH){
      updateOdomPath();
        if (timeToPublish){
        odom_path_publisher_->publish(odomPath);
        if(RUN_TYPE==runTypeList::fullSimul) timeToPublish=false;
      }
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

 bool checkErroneousCycle(double newYaw){

    //prevent lockup from a single erroneous cycle
    if(isnan(newOdom.pose.pose.position.x) || isnan(newOdom.pose.pose.position.y) || isnan(newYaw) ){
        return true;
    }
    if(abs(newOdom.pose.pose.position.x-oldOdom.pose.pose.position.x)>2.0 || abs(newOdom.pose.pose.position.x-oldOdom.pose.pose.position.y)>2.0){
      return true;
    }
    return false;

 }

  void estimateVelocityUsingTicks(double deltaT){

    double leftWheelSpeed=((deltaLeftTicks/deltaT) *2*PI) / TICKSPERWHEELREV;
    double rightWheelSpeed=((deltaRightTicks/deltaT) *2*PI) / TICKSPERWHEELREV;

    newOdom.twist.twist.linear.x = (leftWheelSpeed+rightWheelSpeed)*WHEEL_RADIUS/2;
    newOdom.twist.twist.linear.y = 0;
    newOdom.twist.twist.angular.z = (rightWheelSpeed-leftWheelSpeed)*WHEEL_RADIUS/WHEEL_BASE;

  };

  void odomAlgorithm(){ 
    
    calculatingOdom=true;

    double deltaS_R = (2*PI*WHEEL_RADIUS/(static_cast<float>(TICKSPERWHEELREV))) * (E_D*static_cast<float>(deltaRightTicks));
    double deltaS_L = (2*PI*WHEEL_RADIUS/(static_cast<float>(TICKSPERWHEELREV))) * (static_cast<float>(deltaLeftTicks));

    //local displacement in this time step
    double deltaS = (deltaS_R+deltaS_L)/2;

    //local displacement in this time step (it should be asin of this, but assuming small angular displacement)
    double deltaTheta = (deltaS_R-deltaS_L)/(E_B * WHEEL_BASE);

    //double tempAngle = oldYawEuler + deltaTheta/2;
    deltaS/=1.0798;
     if(deltaTheta>0){
      deltaTheta/=1.0702;
    }


    //calculate new x, y, and theta
    double newYawEuler = deltaTheta + oldYawEuler;
    newOdom.pose.pose.position.x = oldOdom.pose.pose.position.x + cos(newYawEuler)*deltaS;
    newOdom.pose.pose.position.y = oldOdom.pose.pose.position.y + sin(newYawEuler)*deltaS;  

    //prevent lockup from a single erroneous cycle
    if(!checkErroneousCycle(newYawEuler)){

      tf2::Quaternion q;
      q.setRPY(0, 0, newYawEuler);
      q.normalize();
      newOdom.pose.pose.orientation.x = q.x();
      newOdom.pose.pose.orientation.y = q.y();
      newOdom.pose.pose.orientation.z = q.z();
      newOdom.pose.pose.orientation.w = q.w();

      //estimate velocity
      double deltaT=(newTime.nanoseconds() - oldTime.nanoseconds())*1e-9;
      if(deltaT>0.0001){//avoid weird time behaviour in simulation
/*         newOdom.twist.twist.linear.x = deltaS/deltaT;
        newOdom.twist.twist.linear.y = 0.0;
        newOdom.twist.twist.angular.z = deltaTheta/deltaT; */
        estimateVelocityUsingTicks(deltaT);
      }

      if (DYNAMIC_COVARIANCE){
        updateOdomPoseCovariance(deltaS_R, deltaS_L, deltaS, deltaTheta);
        updateOdomTwistCovariance(deltaT);
        oldOdom.pose.covariance = newOdom.pose.covariance;
      }
      //new becomes old
      oldYawEuler=newYawEuler;
      oldOdom.pose.pose.position.x = newOdom.pose.pose.position.x;
      oldOdom.pose.pose.position.y = newOdom.pose.pose.position.y;
      oldTime = newTime;
      oldOdom.header.stamp = newOdom.header.stamp;
    }else{
      //ignore this iteration
      newOdom.pose.pose.position.x = oldOdom.pose.pose.position.x;
      newOdom.pose.pose.position.y = oldOdom.pose.pose.position.y;
    }
    calculatingOdom=false;
  };


  void publishJointState(){

    float left_side_position= static_cast<float>(newLticks)/static_cast<float>(TICKSPERWHEELREV)*2.0*PI;
    float right_side_position=static_cast<float>(newRticks)/static_cast<float>(TICKSPERWHEELREV)*2.0*PI;

    traxter_joints.position.clear();
    traxter_joints.position={left_side_position,right_side_position,left_side_position,right_side_position};
    traxter_joints.header.stamp=newOdom.header.stamp;
    joint_state_publisher_->publish(traxter_joints); 

  };

 void updateOdomPath(){
    geometry_msgs::msg::PoseStamped tempPose;
    tempPose.header=newOdom.header;
    tempPose.pose.position.x = newOdom.pose.pose.position.x;
    tempPose.pose.position.y = newOdom.pose.pose.position.y;
    odomPath.header=newOdom.header;
    odomPath.poses.push_back(tempPose);
  };

  void updateOdomPoseCovariance(double delta_right, double delta_left, double deltaS, double updateAngle){

    motion_increment_covar[0][0] = K_R*abs(delta_right);
    motion_increment_covar[1][1] = K_L*abs(delta_left);
    pose_jacobian[0][2] = -deltaS*sin(updateAngle);
    pose_jacobian[1][2] = deltaS*cos(updateAngle);
    motion_increment_jacobian[0][0] = 0.5*cos(updateAngle) - deltaS/(2*effectiveWheelBase)*sin(updateAngle);
    motion_increment_jacobian[0][1] = 0.5*cos(updateAngle) + deltaS/(2*effectiveWheelBase)*sin(updateAngle);
    motion_increment_jacobian[1][0] = 0.5*sin(updateAngle) + deltaS/(2*effectiveWheelBase)*cos(updateAngle);
    motion_increment_jacobian[1][1] = 0.5*sin(updateAngle) - deltaS/(2*effectiveWheelBase)*cos(updateAngle);

    for (size_t k = 0; k < 3; k++){
      for (size_t i = 0; i <= k; i++){ //covariance matrix is symetric
        
        double tempA=0;
        double tempB=0;
        
        for (size_t j = 0; j < 3; j++){

          tempA += (pose_jacobian[i][0]*oldOdom.pose.covariance[mapCov[0][j]] +
                      pose_jacobian[i][1]*oldOdom.pose.covariance[mapCov[1][j]] +
                      pose_jacobian[i][2]*oldOdom.pose.covariance[mapCov[2][j]]) *
                      pose_jacobian[k][j];
          
          if (j<2){
            tempB += (motion_increment_jacobian[i][0]*motion_increment_covar[0][j] +
                        motion_increment_jacobian[i][1]*motion_increment_covar[1][j]) *
                        motion_increment_jacobian[k][j];
          }
        }
        newOdom.pose.covariance[mapCov[i][k]]=tempA+tempB ;
        if (abs(newOdom.pose.covariance[mapCov[i][k]])>abs(limit_covariance[i][k])){
          newOdom.pose.covariance[mapCov[i][k]]= ((newOdom.pose.covariance[mapCov[i][k]]>=0)+0.0)*limit_covariance[i][k];
        }
        if (i!=k){
          newOdom.pose.covariance[mapCov[k][i]]=newOdom.pose.covariance[mapCov[i][k]];
        }
      } 
    }   
  };

  void updateOdomTwistCovariance(double deltaT){

    for (size_t k = 0; k < 3; k++){
      for (size_t i = 0; i <= k; i++){ //covariance matrix is symetric
        newOdom.twist.covariance[mapCov[i][k]] = (newOdom.pose.covariance[mapCov[i][k]]-oldOdom.pose.covariance[mapCov[i][k]])/deltaT;
        switch (i)
        {
        case 2:
          newOdom.twist.covariance[mapCov[i][k]] = std::max(newOdom.twist.covariance[mapCov[i][k]], 
                                                      0.5*(newOdom.twist.covariance[mapCov[i][k]] + 
                                                      newOdom.twist.twist.angular.z * limit_covariance[i][k])); 
          break;
        default:
          newOdom.twist.covariance[mapCov[i][k]] = std::max(newOdom.twist.covariance[mapCov[i][k]], 
                                                      0.5*(newOdom.twist.covariance[mapCov[i][k]] + 
                                                      newOdom.twist.twist.linear.x * limit_covariance[i][k]));
          break;
        }
        if (i!=k){
          newOdom.twist.covariance[mapCov[k][i]]=newOdom.twist.covariance[mapCov[i][k]];
        }
      }
    }    
  };

  void publish_timer_callback(){
    timeToPublish=true;
  };

  void publish_transform(){
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header= newOdom.header;
    t.header.stamp=this->now();
    t.child_frame_id = "base_link";

    // Turtle only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0
    t.transform.translation.x = newOdom.pose.pose.position.x;
    t.transform.translation.y = newOdom.pose.pose.position.y;
    t.transform.translation.z = 0.0;

    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    t.transform.rotation.x = newOdom.pose.pose.orientation.x;
    t.transform.rotation.y = newOdom.pose.pose.orientation.y;
    t.transform.rotation.z = newOdom.pose.pose.orientation.z;
    t.transform.rotation.w = newOdom.pose.pose.orientation.w;

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }

  std::vector<std::vector<double>> parseVVF(const std::string & input, std::string & error_return)
  {
    std::vector<std::vector<double>> result;

    std::stringstream input_ss(input);
    int depth = 0;
    std::vector<double> current_vector;
    while (!!input_ss && !input_ss.eof()) {
      switch (input_ss.peek()) {
      case EOF:
        break;
      case '[':
        depth++;
        if (depth > 2) {
          error_return = "Array depth greater than 2";
          return result;
        }
        input_ss.get();
        current_vector.clear();
        break;
      case ']':
        depth--;
        if (depth < 0) {
          error_return = "More close ] than open [";
          return result;
        }
        input_ss.get();
        if (depth == 1) {
          result.push_back(current_vector);
        }
        break;
      case ',':
      case ' ':
      case '\t':
        input_ss.get();
        break;
      default:  // All other characters should be part of the numbers.
        if (depth != 2) {
          std::stringstream err_ss;
          err_ss << "Numbers at depth other than 2. Char was '" << char(input_ss.peek()) << "'.";
          error_return = err_ss.str();
          return result;
        }
        double value;
        input_ss >> value;
        if (!!input_ss) {
          current_vector.push_back(value);
        }
        break;
      }
    }
    if (depth != 0) {
      error_return = "Unterminated vector string.";
    } else {
      error_return = "";
    }
    return result;
  };


  rclcpp::Subscription<traxter_msgs::msg::DualMotorArray>::SharedPtr hardware_subscription_;
  rclcpp::Subscription<traxter_msgs::msg::DualMotorArray>::SharedPtr inLoop_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odom_path_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr simul_subscription_;
  rclcpp::TimerBase::SharedPtr publish_timer_;



  nav_msgs::msg::Odometry newOdom;
  nav_msgs::msg::Odometry oldOdom;
  sensor_msgs::msg::JointState traxter_joints;
  nav_msgs::msg::Path odomPath;
  const double PI = 3.1415926535;

  enum runTypeList {fullSimul,inLoop,realHard};

  double oldYawEuler;

  int newLticks=0;
  int newRticks=0;
  int oldLticks=0;
  int oldRticks=0;
  int deltaLeftTicks=0;
  int deltaRightTicks=0;

  double effectiveWheelBase;

  double motion_increment_covar[2][2] = { {0.0, 0.0}, {0.0, 0.0}};
  double pose_jacobian[3][3] = { {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
  int mapCov[3][3] = { {0, 1, 5}, {6, 7, 11}, {30, 31, 35}};
  double motion_increment_jacobian[3][2];


  u_int8_t status;
  const int imin = std::numeric_limits<int>::min(); // minimum value (-2147483647)
  const int imax = std::numeric_limits<int>::max(); // maximum value (2147483647)

  bool calculatingOdom=false;
  bool timeToUpdate=false;
  bool timeToPublish=true;
  bool covarianceLimit=false;

  bool isFirst=true;
  bool encoderFailure=false;

  std::vector<std::vector<double>> limit_covariance;

//parameter holders
  int TICKSPERWHEELREV;
  double WHEEL_RADIUS;
  double WHEEL_BASE;
  int RUN_TYPE;
  double INITIAL_X;
  double INITIAL_Y;
  double INITIAL_THETA;
  double K_R;
  double K_L;
  double E_D;
  double E_B;
  bool PUBLISH_PATH;
  bool DYNAMIC_COVARIANCE;
  bool PUBLISH_TF;
  std::string LIMIT_COVARIANCE;

  //time instances because rclcpp is a mess with time and header stamps
  rclcpp::Time oldTime;
  rclcpp::Time newTime;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Create a loop that will keep the program in execution
  rclcpp::spin(std::make_shared<odometryPublisher>());
  rclcpp::shutdown();
  return 0;
}
