

#include "rclcpp/rclcpp.hpp"

#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using std::placeholders::_1;


class ToolboxHelper : public rclcpp::Node
{
public:
  // Initiate a Node called 'simple_subscriber'
  ToolboxHelper()
  : Node("toolbox_helper")
  {

    publisher_ = this->create_publisher<nav_msgs::msg::Path>("globalpath",1);
    bag_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
          "slam_toolbox/graph_visualization", 2, std::bind(&ToolboxHelper::array_callback, this, _1));


    }


private:
  // Define a function called 'topic_callback' that receives a parameter named 'msg' 

void array_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg){

    nav_msgs::msg::Path outpath;
    geometry_msgs::msg::PoseStamped temp;

for (size_t i = 0; i < msg->markers.size(); i++)
{
    if (msg->markers[i].type==2)
    {
        temp.pose.position.x=msg->markers[i].pose.position.x;
        temp.pose.position.y=msg->markers[i].pose.position.y;
        outpath.poses.push_back(temp);
        
        
    }
    
}

    outpath.header.frame_id="map";
    publisher_->publish(outpath);
}


  

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr bag_subscription_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  };



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Create a loop that will keep the program in execution
  rclcpp::spin(std::make_shared<ToolboxHelper>());
  rclcpp::shutdown();
  return 0;
}

