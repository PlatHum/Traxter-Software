

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>
#include <traxter_msgs/msg/light_imu.hpp>
#include "sensor_msgs/msg/imu.hpp"
#include <tf2/LinearMath/Quaternion.h>

#include <cstdio>  // for EOF
#include <string>
#include <sstream>
#include <vector>
#include <math.h>
using std::placeholders::_1;

class imuInterpreter : public rclcpp::Node
{
public:
  // Initiate a Node called 'simple_subscriber'
  imuInterpreter()
  : Node("imu_interpreter")
  {

    this->declare_parameter("_run_type", 2);//0 full simulation, 1 hardware in the loop, 2 real hardware
    this->declare_parameter("_orientation_covariance", "[[0.0, 0.0,  0.0], [0.0, 0.0,  0.0], [0.0, 0.0,  0.0001826]]");
    this->declare_parameter("_angular_velocity_covariance", "[[2.439e-05, 5.169e-07,  -2.577e-07], [5.169e-07, 6.305e-06,  -2.786e-07], [-2.577e-07, -2.786e-07,  2.267e-05]]");
    this->declare_parameter("_linear_acceleration_covariance", "[[0.0001661, -9.742e-06,  -2.256e-05], [-9.742e-06, 0.0001219,  2.224e-05], [-2.256e-05, 2.224e-05,  0.0004139]]");


    rclcpp::QoS qos(3);
    qos.keep_last(3);
    qos.best_effort();
    qos.durability_volatile();


    this->get_parameter("_run_type", RUN_TYPE);
    this->get_parameter("_orientation_covariance", ORIENTATION_COVARIANCE);
    this->get_parameter("_angular_velocity_covariance", ANGULAR_VELOCITY_COVARIANCE);
    this->get_parameter("_linear_acceleration_covariance", LINEAR_ACCELERATION_COVARIANCE);

    switch (RUN_TYPE)
    {
    case runTypeList::fullSimul:
        RCLCPP_INFO(this->get_logger(), "IMU interpretation running in full Simulation mode.");
        simul_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
          "traxter/simulation/imu", 2, std::bind(&imuInterpreter::simulation_topic_callback, this, _1));
      break;
    case runTypeList::inLoop: //hardware in the loop
        RCLCPP_INFO(this->get_logger(), "IMU interpretation running in Hardware in the Loop mode.");
      hardware_subscription_ = this->create_subscription<traxter_msgs::msg::LightImu>(
        "traxter/imu/data/unprocessed", qos, std::bind(&imuInterpreter::hardware_topic_callback, this, _1));
      break;
    case runTypeList::realHard: //hardware
        RCLCPP_INFO(this->get_logger(), "IMU interpretation running in Hardware mode.");

      hardware_subscription_ = this->create_subscription<traxter_msgs::msg::LightImu>(
        "traxter/imu/data/unprocessed", qos, std::bind(&imuInterpreter::hardware_topic_callback, this, _1));
      break;
    default:
        RCLCPP_FATAL(this->get_logger(), "Could not interpret run type. Could not construct IMU interpreter node.");
      break;
    }

    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu",1);
    standard_imu_message.header.frame_id="imu_link";
    std::string ERROR;
    orientation_covariance = parseVVF(ORIENTATION_COVARIANCE, ERROR);
    angular_velocity_covariance = parseVVF(ANGULAR_VELOCITY_COVARIANCE, ERROR);
    linear_acceleration_covariance = parseVVF(LINEAR_ACCELERATION_COVARIANCE, ERROR);

    if(RUN_TYPE!=runTypeList::fullSimul){
      for (size_t i = 0; i < 3; i++){
        for (size_t j = 0; j <= i; j++){
          standard_imu_message.orientation_covariance[mapCov[i][j]]=orientation_covariance[i][j];
          standard_imu_message.angular_velocity_covariance[mapCov[i][j]]=angular_velocity_covariance[i][j];
          standard_imu_message.linear_acceleration_covariance[mapCov[i][j]]=linear_acceleration_covariance[i][j];
          if (i!=j){
            standard_imu_message.orientation_covariance[mapCov[j][i]]=standard_imu_message.orientation_covariance[mapCov[i][j]];
            standard_imu_message.angular_velocity_covariance[mapCov[j][i]]=standard_imu_message.angular_velocity_covariance[mapCov[i][j]];
            standard_imu_message.linear_acceleration_covariance[mapCov[j][i]]=standard_imu_message.linear_acceleration_covariance[mapCov[i][j]];
          }
        }
      }
    }
  }

private:
  // Define a function called 'topic_callback' that receives a parameter named 'msg' 
  void hardware_topic_callback(const traxter_msgs::msg::LightImu::SharedPtr msg)
  {

    tf2::Quaternion q((msg->q1)/100.0,(msg->q2)/100.0,(msg->q3)/100.0,(msg->q0)/100.0);
    q.normalize();

    standard_imu_message.orientation.x= q.x();
    standard_imu_message.orientation.y= q.y();
    standard_imu_message.orientation.z= q.z();
    standard_imu_message.orientation.w= q.w();

    standard_imu_message.angular_velocity.x=msg->gyrox/100.0;
    standard_imu_message.angular_velocity.y=msg->gyroy/100.0;
    standard_imu_message.angular_velocity.z=msg->gyroz/100.0;
    standard_imu_message.linear_acceleration.x=msg->accx/100.0;
    standard_imu_message.linear_acceleration.y=msg->accy/100.0;
    standard_imu_message.linear_acceleration.z=msg->accz/100.0;
    standard_imu_message.header.stamp=this->now();

    publisher_->publish(standard_imu_message);

    if(msg->gyro_status<3){
      uncalibGyro++;
      if(uncalibGyro>100){
        RCLCPP_WARN(this->get_logger(), "Gyroscope not fully calibrated! Level: %d/3",msg->gyro_status);
        uncalibGyro=0;
      }
    }
    if(msg->acc_status<3){
      uncalibAccel++;
      if(uncalibAccel>100){
        RCLCPP_WARN(this->get_logger(), "Accelerometer not fully calibrated! Level: %d/3",msg->acc_status);
        uncalibAccel=0;
      }    
    }
    if(msg->mag_status<3){
      uncalibMag++;
      if(uncalibMag>100){
        RCLCPP_WARN(this->get_logger(), "Magnetometer not fully calibrated! Level: %d/3",msg->mag_status);
        uncalibMag=0;
      }    
    }

      if(msg->mag_status>5 || msg->acc_status>5 || msg->gyro_status>5){

        RCLCPP_FATAL(this->get_logger(), "IMU NOT CONNECTED PROPERLY!");

      }    
  }


void simulation_topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg){

    if(isFirst){
      previousTime= rclcpp::Time(msg->header.stamp.sec , msg->header.stamp.nanosec);
      currentTime= previousTime;
      isFirst=false;
    }else{
      currentTime = rclcpp::Time(msg->header.stamp.sec , msg->header.stamp.nanosec);
    }

      double deltaT= (currentTime-previousTime).nanoseconds()*1e-9;
      double pitch = prevPitch + deltaT * 0.5 * (msg->angular_velocity.y + standard_imu_message.angular_velocity.y);
      double roll = prevRoll + deltaT * 0.5 * (msg->angular_velocity.x + standard_imu_message.angular_velocity.x);
      double yaw = prevYaw + deltaT * 0.5 * (msg->angular_velocity.z + standard_imu_message.angular_velocity.z);
      tf2::Quaternion q;
      q.setRPY(roll, pitch, yaw);
      q.normalize();
      msg->orientation.x= q.x();
      msg->orientation.y= q.y();
      msg->orientation.z= q.z();
      msg->orientation.w= q.w();

      for (size_t i = 0; i < 3; i++){
        for (size_t j = 0; j <= i; j++){
          msg->orientation_covariance[mapCov[i][j]]=orientation_covariance[i][j];
          msg->angular_velocity_covariance[mapCov[i][j]]=angular_velocity_covariance[i][j];
          msg->linear_acceleration_covariance[mapCov[i][j]]=linear_acceleration_covariance[i][j];
          if (i!=j){
            msg->orientation_covariance[mapCov[j][i]]=msg->orientation_covariance[mapCov[i][j]];
            msg->angular_velocity_covariance[mapCov[j][i]]=msg->angular_velocity_covariance[mapCov[i][j]];
            msg->linear_acceleration_covariance[mapCov[j][i]]=msg->linear_acceleration_covariance[mapCov[i][j]];
          }
        }
      }
      previousTime = currentTime;
      msg->header.frame_id="imu_link";

      if(cycles>29){
        publisher_->publish(*msg);
      }else{
        cycles++;
      }

  standard_imu_message.angular_velocity.x=msg->angular_velocity.x;
  standard_imu_message.angular_velocity.y=msg->angular_velocity.y;
  standard_imu_message.angular_velocity.z=msg->angular_velocity.z;
  prevRoll = roll;
  prevPitch = pitch;
  prevYaw = yaw;
  

};


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

  rclcpp::Subscription<traxter_msgs::msg::LightImu>::SharedPtr hardware_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr simul_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  sensor_msgs::msg::Imu standard_imu_message;
  bool isFirst=true;
  int biasQuat[4];
  enum runTypeList {fullSimul,inLoop,realHard};
  std::string ORIENTATION_COVARIANCE;
  std::string ANGULAR_VELOCITY_COVARIANCE;
  std::string LINEAR_ACCELERATION_COVARIANCE;
  int RUN_TYPE;
  int mapCov[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}}; 
  std::vector<std::vector<double>> orientation_covariance;
  std::vector<std::vector<double>> angular_velocity_covariance;
  std::vector<std::vector<double>>linear_acceleration_covariance;
  rclcpp::Time currentTime;
  rclcpp::Time previousTime;
  double prevPitch = 0;
  double prevRoll = 0;
  double prevYaw = 0;
  int cycles=0;
  uint8_t uncalibMag=0;
  uint8_t uncalibAccel=0;
  uint8_t uncalibGyro=0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Create a loop that will keep the program in execution
  rclcpp::spin(std::make_shared<imuInterpreter>());
  rclcpp::shutdown();
  return 0;
}

