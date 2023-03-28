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

class image_to_tf
{
public:
  image_to_tf();
  ~image_to_tf();

private:
  void gps_to_tf(const sensor_msgs::NavSatFixConstPtr& gps,
                 const sensor_msgs::ImuConstPtr& imu,
                 const geometry_msgs::Vector3StampedConstPtr& angle);

  void tf_to_LonLat_simple(const tf2::Vector3& input_tf, float& Lon , float& Lat);
  void LonLat_to_tf_simple(const float& Lon ,const float& Lat ,const float& Att, tf2::Vector3& output_tf);



  ros::NodeHandle nh_;
  ros::NodeHandle pnh;

  ros::Publisher marker_pub;
  ros::Publisher marker_pub_p;
  ros::Publisher marker_pub_h;
  ros::Publisher plane_pub;
  ros::Publisher save_file_pub;
  ros::Publisher yolo_starter_pub;

  ros::Subscriber camera_th_sub;
  
  //ros::Subscriber yolo_topic_sub_;

  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  //image_transport::Publisher image_pub_;

  typedef message_filters::TimeSynchronizer<
      sensor_msgs::CameraInfo, sensor_msgs::Image >
      SyncSubscriber;

  message_filters::Subscriber< sensor_msgs::CameraInfo > camera_sub_;
  message_filters::Subscriber< sensor_msgs::Image > img_sub_;
  boost::scoped_ptr< SyncSubscriber > sync_sub_;



  typedef message_filters::TimeSynchronizer<
      sensor_msgs::Image, vision_msgs::Detection2DArray >
      SyncSubscriber_yolo;

  message_filters::Subscriber< sensor_msgs::Image > img_yolo_sub_;
  message_filters::Subscriber< vision_msgs::Detection2DArray > msgs_yolo_sub_;
  boost::scoped_ptr< SyncSubscriber_yolo > sync_yolo_sub_;


  typedef message_filters::TimeSynchronizer<
      sensor_msgs::NavSatFix, sensor_msgs::Imu >
      SyncSubscriber_tfbr;

  message_filters::Subscriber< sensor_msgs::NavSatFix > gps_sub_;
  message_filters::Subscriber< sensor_msgs::Imu > imu_sub_;
  message_filters::Subscriber< geometry_msgs::Vector3Stamped > angle_sub_;
  boost::scoped_ptr< SyncSubscriber_tfbr > sync_tfbr_sub_;  

  float c_x,c_y;
  float Score[1000][1000];
  float head[5];
  float magnification;
  float camera_cx,camera_cy,camera_fx,camera_fy;
  

  std::string frame_id,camera_tf,map_tf,file_path,drone_tf;
  int col_px,row_px;

  geometry_msgs::TransformStamped transformStamped;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;

  jsk_recognition_msgs::SimpleOccupancyGrid plane_;
  jsk_recognition_msgs::SimpleOccupancyGridArray plane_input;
  tf2_ros::TransformBroadcaster dynamic_br_;
  tf2_ros::TransformBroadcaster dynamic_high_;
  tf2_ros::TransformBroadcaster dynamic_br_drone;
  float origin_Lat;
  float origin_Lon;
  float earth_R;
  float thresh_humandetection,magin_high,start_high;

  visualization_msgs::Marker line_strip_high;
  //sensor_msgs::CameraInfoConstPtr camera_msg_th;


  std::string write_file_path;
  
  int file_numbering;
  int yolo_detection_number;

  tf2::Transform nav_to_dronexy;
  tf2::Transform nav_to_drone_gps;
  cv::Mat original_image;
  tf2_ros::TransformBroadcaster camera_tf_br;
  std::ofstream writing_file;
  std::string file_csv;
  float gps_lon,gps_lat;

  tf2_ros::TransformBroadcaster map_camera_br;
  tf2_ros::TransformBroadcaster map_drone_br;

};



image_to_tf::image_to_tf():
  pnh("~"),it_(nh_),tfListener(tfBuffer),file_numbering(0),yolo_detection_number(0)
{

  pnh.param("frame_id", frame_id,std::string("ardrone_base_bottomcam_plugin"));
  pnh.param("col_px", col_px,0);
  pnh.param("row_px", row_px,0);
  pnh.param("camera_tf", camera_tf,std::string("odom"));
  pnh.param("map_tf", map_tf,std::string("start_point"));
  pnh.param("file_path", file_path,std::string("/media/data/drone_ws/src/gk_drone_pkg/script/test_10000.txt"));

  pnh.param("origin_Lat", origin_Lat, float(43.28936));  //startpoint lat 
  pnh.param("origin_Lon", origin_Lon, float(143.23502));  //startpoint lon
  pnh.param("earth_R", earth_R, float(6377397.155));

  pnh.param("thresh_humandetection", thresh_humandetection, float(0.001));
  pnh.param("magin_high", magin_high, float(639.000000));
  pnh.param("magnification", magnification, float(28));
  pnh.param("write_file_path", write_file_path,std::string("/media/data/test_data"));
  pnh.param("drone_tf", drone_tf,std::string("drone"));

  pnh.param("camera_cx", camera_cx,float(374.67));
  pnh.param("camera_cy", camera_cy,float(374.67));
  pnh.param("camera_fx", camera_fx,float(320.5));
  pnh.param("camera_fy", camera_fy,float(180.5));
  pnh.param("start_high", start_high,float(314.6645));



  gps_sub_.subscribe(nh_, "dji_osdk_ros/gps_position", 1);
  imu_sub_.subscribe(nh_, "dji_osdk_ros/imu", 1);
  angle_sub_.subscribe(nh_, "dji_osdk_ros/gimbal_angle", 1);
  
  const int queue_size(pnh.param("queue_size", 10));

  sync_tfbr_sub_.reset(new SyncSubscriber_tfbr(queue_size));
  sync_tfbr_sub_->connectInput(gps_sub_, imu_sub_,angle_sub_);
  sync_tfbr_sub_->registerCallback(&image_to_tf::gps_to_tf, this);

}

image_to_tf::~image_to_tf()
  {
  }

void image_to_tf::gps_to_tf(const sensor_msgs::NavSatFixConstPtr& gps,
                            const sensor_msgs::ImuConstPtr& imu ,
                            const geometry_msgs::Vector3StampedConstPtr& angle)

  {
      tf2::Quaternion quat_tf;
      quat_tf.setRPY(angle->vector.x * 0.01745329, angle->vector.y * 0.01745329, angle->vector.z * 0.01745329);
      geometry_msgs::Quaternion quat_msg;
      tf2::convert(quat_tf, quat_msg);

      tf2::Quaternion quat_camera_prefix;
      quat_camera_prefix.setRPY(0,-1.57,0);
      geometry_msgs::Quaternion camera_prefix_msg;
      tf2::convert(quat_camera_prefix, camera_prefix_msg);



      tf2::Vector3 gps_to_tf_pose;
      tf2::Quaternion gps_to_tf_att;
      
      image_to_tf::LonLat_to_tf_simple(gps->longitude ,gps->latitude ,gps->altitude,gps_to_tf_pose);
      nav_to_drone_gps.setOrigin(gps_to_tf_pose);
      
      tf2::convert(imu->orientation , gps_to_tf_att);

      geometry_msgs::TransformStamped map_cam_msg;
      map_cam_msg.header.stamp = ros::Time::now();
      map_cam_msg.header.frame_id = map_tf;
      map_cam_msg.child_frame_id = "gimbal";
      map_cam_msg.transform.translation.x = gps_to_tf_pose.getX();
      map_cam_msg.transform.translation.y = gps_to_tf_pose.getY();
      map_cam_msg.transform.translation.z = gps_to_tf_pose.getZ();
      map_cam_msg.transform.rotation =quat_msg;
      map_camera_br.sendTransform(map_cam_msg);

      geometry_msgs::TransformStamped cam_pic_msg;
      map_cam_msg.header.stamp = ros::Time::now();
      map_cam_msg.header.frame_id = "gimbal";
      map_cam_msg.child_frame_id = camera_tf;
      map_cam_msg.transform.translation.x = 0;
      map_cam_msg.transform.translation.y = 0;
      map_cam_msg.transform.translation.z = 0;
      map_cam_msg.transform.rotation =camera_prefix_msg;
      map_camera_br.sendTransform(map_cam_msg);


      geometry_msgs::TransformStamped map_drone_msg;
      map_drone_msg.header.stamp = ros::Time::now();
      map_drone_msg.header.frame_id =map_tf;
      map_drone_msg.child_frame_id = drone_tf;
      map_drone_msg.transform.translation.x = gps_to_tf_pose.getX();
      map_drone_msg.transform.translation.y = gps_to_tf_pose.getY();
      map_drone_msg.transform.translation.z = gps_to_tf_pose.getZ();
      map_drone_msg.transform.rotation =imu->orientation;
      map_drone_br.sendTransform(map_drone_msg);

      nav_to_drone_gps.setRotation(gps_to_tf_att);
  }


void image_to_tf::tf_to_LonLat_simple(const tf2::Vector3& input_tf, float& Lon , float& Lat)
  {
    float R_Lat = earth_R * std::cos(gps_lat);
    Lon = (input_tf.getX()+R_Lat*0.01747737*gps_lon)/(R_Lat*0.01747737);
    Lat = (input_tf.getY()+earth_R*0.01747737*gps_lat)/(earth_R*0.01747737);

    //ROS_INFO("tf1 R_Lat:%f, Lon:%f ,dis:%f", R_Lat,Lon,R_Lat*0.01747737*origin_Lon);
  }

void image_to_tf::LonLat_to_tf_simple(const float& Lon ,const float& Lat ,const float& Att, tf2::Vector3& output_tf)
  {
    float R_Lat = earth_R * std::cos(origin_Lat);
    float x = R_Lat *0.01747737 * (Lon-origin_Lon);
    float y = earth_R *0.01747737 * (Lat-origin_Lat);
    float z = Att-magin_high  ;
    output_tf.setValue(x,y,z); 
  }




int main(int argc, char** argv)
{
  ros::init(argc, argv, "joycon");
  image_to_tf joy_ctrl_megarover;
  
  ros::NodeHandle n;

	ros::Rate r(10);
  while(n.ok()){

  	ros::spinOnce();
		r.sleep();
	}
}
