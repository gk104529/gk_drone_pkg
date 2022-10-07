#include <ros/ros.h>
#include <cstdio>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
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
#include <sensor_msgs/Imu.h>
//#include <jsk_rviz_plugins/OverlayMenu.h>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <math.h>
#include <vision_msgs/Detection2DArray.h>
#include "std_msgs/String.h"


class tf_setter
{
public:
  tf_setter();

private:
  void tf_broadcaster(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& input_pose);
  void drone_tf_setter(const geometry_msgs::PointStampedConstPtr& point,
                const sensor_msgs::ImuConstPtr& imu);

  void camera_tf_setter(const geometry_msgs::Vector3StampedConstPtr& angle );

  void tf_to_LonLat_simple(const tf2::Vector3& input_tf, float& Lon , float& Lat);
  void LonLat_to_tf_simple(const float& Lon ,const float& Lat ,const float& Att, tf2::Vector3& output_tf);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh;
  ros::Subscriber camera_th_sub;

  std::string parent_tf,child_tf;
  tf2_ros::TransformBroadcaster drone_tf_br;
  tf2_ros::TransformBroadcaster camera_tf_br;

  typedef message_filters::TimeSynchronizer<
      geometry_msgs::PointStamped, sensor_msgs::Imu >
      SyncSubscriber;

  message_filters::Subscriber< geometry_msgs::PointStamped > d_position_sub_;
  message_filters::Subscriber< sensor_msgs::Imu > d_imu_sub;
  boost::scoped_ptr< SyncSubscriber > sync_sub_;

  std::string drone_start_tf;
  std::string drone_tf;
  std::string camera_tf;

};


tf_setter::tf_setter():
  pnh("~")
{

  pnh.param("drone_start_tf", drone_start_tf,std::string("start"));
  pnh.param("drone_tf", drone_tf,std::string("drone"));
  pnh.param("camera_tf", camera_tf,std::string("camera"));

  pnh.param("origin_lon", origin_lon,0.0);
  pnh.param("origin_lat", origin_lat,0.0);

  d_position_sub_.subscribe(nh_, "dji_osdk_ros/local_position", 1);
  d_imu_sub.subscribe(nh_, "dji_osdk_ros/imu", 1);

  const int queue_size(pnh.param("queue_size", 10));

  sync_sub_.reset(new SyncSubscriber(queue_size));
  sync_sub_->connectInput(d_position_sub_, d_imu_sub);
  sync_sub_->registerCallback(&tf_setter::drone_tf_setter, this);

  camera_th_sub  = nh_.subscribe("dji_osdk_ros/gimbal_angle", 10, &tf_setter::camera_tf_setter,this );


 
}

void tf_setter::drone_tf_setter(const geometry_msgs::PointStampedConstPtr& point ,
                         const sensor_msgs::ImuConstPtr& imu)
  {   
      
      geometry_msgs::TransformStamped transformStamped2;
      transformStamped2.header.stamp = ros::Time::now();
      transformStamped2.header.frame_id = drone_start_tf;
      transformStamped2.child_frame_id = drone_tf;
      transformStamped2.transform.translation.x = point->point.x;
      transformStamped2.transform.translation.y = point->point.x;
      transformStamped2.transform.translation.z = point->point.x;
      transformStamped2.transform.rotation = imu->orientation;
      drone_tf_br.sendTransform(transformStamped2);
  }

void tf_setter::camera_tf_setter(const geometry_msgs::Vector3StampedConstPtr& angle )
  {   
      tf2::Quaternion quat_tf;
      quat_tf.setRPY(angle->vector.x, angle->vector.y, angle->vector.z);
      geometry_msgs::Quaternion quat_msg;
      tf2::convert(quat_tf, quat_msg);

      geometry_msgs::TransformStamped transformStamped2;
      transformStamped2.header.stamp = ros::Time::now();
      transformStamped2.header.frame_id = drone_tf;
      transformStamped2.child_frame_id = camera_tf;
      transformStamped2.transform.translation.x = 0;
      transformStamped2.transform.translation.y = 0;
      transformStamped2.transform.translation.z = 0;
      transformStamped2.transform.rotation =quat_msg;
      camera_tf_br.sendTransform(transformStamped2);
  }

void image_to_tf::tf_to_LonLat_simple(const tf2::Vector3& input_tf, float& Lon , float& Lat)
  {
    float R_Lat = earth_R * std::cos(origin_Lat);
    Lon = (input_tf.getX()+R_Lat*0.01747737*origin_Lon)/(R_Lat*0.01747737);
    Lat = (input_tf.getY()+earth_R*0.01747737*origin_Lat)/(earth_R*0.01747737);

    //ROS_INFO("tf1 R_Lat:%f, Lon:%f ,dis:%f", R_Lat,Lon,R_Lat*0.01747737*origin_Lon);
  }

void image_to_tf::LonLat_to_tf_simple(const float& Lon ,const float& Lat ,const float& Att, tf2::Vector3& output_tf)
  {
    float R_Lat = earth_R * std::cos(origin_Lat);
    float x = R_Lat *0.01747737 * (Lon-origin_Lon);
    float y = earth_R *0.01747737 * (Lat-origin_Lat);
    float z = Att-31.5991 ;
    output_tf.setValue(x,y,z); 
  }  
  

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joycon");
  tf_setter tf_setting;
  
  ros::NodeHandle n;

	ros::Rate r(10);
  while(n.ok()){

  	ros::spinOnce();
		r.sleep();
	}
}
