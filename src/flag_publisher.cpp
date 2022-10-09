#include <ros/ros.h>
#include <cstdio>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/NavSatFix.h>
#include <turtlesim/Spawn.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <visualization_msgs/Marker.h>
#include <jsk_recognition_msgs/SimpleOccupancyGrid.h>
#include <jsk_recognition_msgs/SimpleOccupancyGridArray.h>
//#include <jsk_rviz_plugins/OverlayMenu.h>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <math.h>
#include <vision_msgs/Detection2DArray.h>
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include <iomanip>
#include <std_msgs/Empty.h>


static const std::string OPENCV_WINDOW = "Image window";

class flag_publisher
{
public:
  flag_publisher();
  ~flag_publisher();

private:


  void camera_tf_setter(const geometry_msgs::Vector3StampedConstPtr& angle );



  ros::NodeHandle nh_;
  ros::NodeHandle pnh;

  ros::Publisher yolo_starter_pub;

  ros::Subscriber camera_th_sub;
  bool yolo_startflag;

};



flag_publisher::flag_publisher():
  pnh("~")
{
  pnh.param("yolo_startflag", yolo_startflag , true);


  yolo_starter_pub = nh_.advertise<std_msgs::Empty>("yolo_flag", 10,this);
  
  camera_th_sub  = nh_.subscribe("dji_osdk_ros/gimbal_angle", 10, &flag_publisher::camera_tf_setter,this );
}

flag_publisher::~flag_publisher()
  {
  }

void flag_publisher::camera_tf_setter(const geometry_msgs::Vector3StampedConstPtr& angle )
  {   
      if (std::abs(angle->vector.y && yolo_startflag ) > 20 ){
        std_msgs::Empty go_yolo;
        yolo_starter_pub.publish(go_yolo);
      }      
  }



int main(int argc, char** argv)
{
  ros::init(argc, argv, "joycon");
  flag_publisher joy_ctrl_megarover;
  
  ros::NodeHandle n;

	ros::Rate r(10);
  while(n.ok()){

  	ros::spinOnce();
		r.sleep();
	}
}
