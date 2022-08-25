#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "RTProtocol.h"
#include "RTPacket.h"

#ifdef _WIN32
#define sleep Sleep
#else
#include <unistd.h>
#endif

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

class MoCapPublisher : public rclcpp::Node
{
  public:
    MoCapPublisher()
    : Node("mocap_publisher")
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("traxter/mocap/pose", 10);
      timer_ = this->create_wall_timer(15ms, std::bind(&MoCapPublisher::timer_callback, this));
    }

  private:
    void timer_callback(){
      //auto message = std_msgs::msg::String();
      //message.data = "Hello, world! " + std::to_string(count_++);
      //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    try
    {
            if (!rtProtocol.Connected())
            {
                if (!rtProtocol.Connect(serverAddr.c_str(), basePort, &udpPort, majorVersion, minorVersion, bigEndian))
                {
                    RCLCPP_ERROR(this->get_logger(),"rtProtocol.Connect: %s", rtProtocol.GetErrorString());
                    sleep(1);
                    return;
                }
            }

            if (!dataAvailable)
            {
                if (!rtProtocol.Read6DOFSettings(dataAvailable))
                {
                    RCLCPP_ERROR(this->get_logger(),"rtProtocol.Read6DOFSettings: %s", rtProtocol.GetErrorString());
                    sleep(1);
                    return;
                }
            }

            if (!streamFrames)
            {
                if (!rtProtocol.StreamFrames(CRTProtocol::RateAllFrames, 0, udpPort, NULL, CRTProtocol::cComponent6d))
                {
                    RCLCPP_ERROR(this->get_logger(),"rtProtocol.StreamFrames: %s", rtProtocol.GetErrorString());
                    sleep(1);
                    return;
                }
                streamFrames = true;

                RCLCPP_INFO(this->get_logger(),"Starting to streaming 6DOF data");
            }

            CRTPacket::EPacketType packetType;

            if (rtProtocol.Receive(packetType, true) == CNetwork::ResponseType::success)
            {
                if (packetType == CRTPacket::PacketData)
                {
                    float fX, fY, fZ;
                    float rotationMatrix[9];

                    CRTPacket* rtPacket = rtProtocol.GetRTPacket();


                    for (unsigned int i = 0; i < rtPacket->Get6DOFBodyCount(); i++)
                    {
                        if (rtPacket->Get6DOFBody(i, fX, fY, fZ, rotationMatrix))
                        {
                            const char* pTmpStr = rtProtocol.Get6DOFBodyName(i);
                            if (strcmp(pTmpStr, bodyName.c_str()))
                            {
                                geometry_msgs::msg::PoseStamped message;
                                message.pose.position.x=fX;
                                message.pose.position.y=fY;
                                //message.pose.position.z=fZ;
                                tf2::Matrix3x3 m(rotationMatrix[0], rotationMatrix[1], rotationMatrix[2],
                                rotationMatrix[3], rotationMatrix[4], rotationMatrix[5], rotationMatrix[6], rotationMatrix[7], rotationMatrix[8]);
                                tf2::Quaternion q;
                                m.getRotation(q);
                                q.normalize();
                                message.pose.orientation.x=q.x();
                                message.pose.orientation.y=q.y();
                                message.pose.orientation.z=q.z();
                                message.pose.orientation.w=q.w();
                                message.header.stamp = this->now();
                                message.header.frame_id="map";
                                publisher_->publish(message);
                                break;
                            }
                            else
                            {
                                continue;
                            }
                        }
                    }
                }
        }
/*         rtProtocol.StopCapture();
        rtProtocol.Disconnect(); */
    }
    catch (std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(),"Error in Qualysis node: %s", e.what());
    }
};
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    
    CRTProtocol rtProtocol;
    const std::string           serverAddr = "127.0.0.1";
    const std::string           bodyName = "traxter";
    const unsigned short basePort = 22222;
    const int            majorVersion = 1;
    const int            minorVersion = 19;
    const bool           bigEndian = false;

    bool dataAvailable = false;
    bool streamFrames = false;
    unsigned short udpPort = 6734;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoCapPublisher>());
  rclcpp::shutdown();
  return 0;
}
