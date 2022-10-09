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
  void imageCb(const sensor_msgs::CameraInfoConstPtr& camera_msg,
               const sensor_msgs::ImageConstPtr& msg);

  void set_line(const cv::Point& point);

  void transformMsgToTF2(const geometry_msgs::Transform& msg, tf2::Transform& tf2);

  void text_reader(const std::string& file_path);

  void tf_to_LonLat_simple(const tf2::Vector3& input_tf, float& Lon , float& Lat);
  void LonLat_to_tf_simple(const float& Lon ,const float& Lat ,const float& Att, tf2::Vector3& output_tf);

  void get_line_high(const geometry_msgs::Point& p_P, geometry_msgs::Point& p_C);

  void yolo_to_tf(const sensor_msgs::ImageConstPtr& msg ,
                  const vision_msgs::Detection2DArrayConstPtr& yolo_msg);

  void file_writter(const tf2::Vector3& target);

  void gps_to_tf(const sensor_msgs::NavSatFixConstPtr& gps,
                 const sensor_msgs::ImuConstPtr& imu); 

  void camera_tf_setter(const geometry_msgs::Vector3StampedConstPtr& angle );



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

};



image_to_tf::image_to_tf():
  pnh("~"),it_(nh_),tfListener(tfBuffer),file_numbering(0),yolo_detection_number(0)
{

  //camera_sub_.subscribe(nh_, "camera_info", 1);
  //img_sub_.subscribe(nh_, "image_row", 1);

  
  img_yolo_sub_.subscribe(nh_, "image_row_yolo", 1);
  msgs_yolo_sub_.subscribe(nh_, "msg_yolo", 1);

  gps_sub_.subscribe(nh_, "dji_osdk_ros/gps_position", 1);
  imu_sub_.subscribe(nh_, "dji_osdk_ros/imu", 1);

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

  text_reader(file_path);


  //image_pub_ = it_.advertise("image_converter/output_video", 1);
  marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  marker_pub_p = nh_.advertise<visualization_msgs::Marker>("visualization_marker_p", 10);
  marker_pub_h = nh_.advertise<visualization_msgs::Marker>("visualization_marker_h", 10);

  plane_pub = nh_.advertise<jsk_recognition_msgs::SimpleOccupancyGridArray>("plane_maker", 10);
  save_file_pub = nh_.advertise<std_msgs::String>("save_file", 10);
  


  const int queue_size(pnh.param("queue_size", 10));

  /*sync_sub_.reset(new SyncSubscriber(queue_size));
  sync_sub_->connectInput(camera_sub_, img_sub_);
  sync_sub_->registerCallback(&image_to_tf::imageCb, this);*/


  sync_yolo_sub_.reset(new SyncSubscriber_yolo(queue_size));
  sync_yolo_sub_->connectInput(img_yolo_sub_, msgs_yolo_sub_);
  sync_yolo_sub_->registerCallback(&image_to_tf::yolo_to_tf, this);

  sync_tfbr_sub_.reset(new SyncSubscriber_tfbr(queue_size));
  sync_tfbr_sub_->connectInput(gps_sub_, imu_sub_);
  sync_tfbr_sub_->registerCallback(&image_to_tf::gps_to_tf, this);

  camera_th_sub  = nh_.subscribe("dji_osdk_ros/gimbal_angle", 10, &image_to_tf::camera_tf_setter,this );

  //gps_sub  = nh_.subscribe("dji_osdk_ros/gps_position", 10, &image_to_tf::gps_to_tf,this );
  //yolo_topic_sub_=nh_.subscribe<vision_msgs::Detection2DArray>("yolo_to_tf", 10, &image_to_tf::yolo_to_tf, this);

}

image_to_tf::~image_to_tf()
  {
  }

void image_to_tf::camera_tf_setter(const geometry_msgs::Vector3StampedConstPtr& angle )
  {   
      tf2::Quaternion quat_tf;
      quat_tf.setRPY(angle->vector.x * 0.01745329, angle->vector.y * 0.01745329, angle->vector.z * 0.01745329);
      

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

void image_to_tf::gps_to_tf(const sensor_msgs::NavSatFixConstPtr& gps,
                            const sensor_msgs::ImuConstPtr& imu )

  {
      tf2::Vector3 gps_to_tf_pose;
      tf2::Quaternion gps_to_tf_att;
      gps_lon=gps->longitude;
      gps_lat=gps->latitude;
      image_to_tf::LonLat_to_tf_simple(gps->longitude ,gps->latitude ,gps->altitude,gps_to_tf_pose);
      nav_to_drone_gps.setOrigin(gps_to_tf_pose);
      
      tf2::convert(imu->orientation , gps_to_tf_att);

      geometry_msgs::TransformStamped transformStamped2;
      transformStamped2.header.stamp = ros::Time::now();
      transformStamped2.header.frame_id =map_tf;
      transformStamped2.child_frame_id = drone_tf;
      transformStamped2.transform.translation.x = gps_to_tf_pose.getX();
      transformStamped2.transform.translation.y = gps_to_tf_pose.getY();
      transformStamped2.transform.translation.z = gps_to_tf_pose.getZ();
      transformStamped2.transform.rotation =imu->orientation;
      camera_tf_br.sendTransform(transformStamped2);

      nav_to_drone_gps.setRotation(gps_to_tf_att);
  }

void image_to_tf::yolo_to_tf(const sensor_msgs::ImageConstPtr& msg ,
                             const vision_msgs::Detection2DArrayConstPtr& yolo_msg)
  {
    if (yolo_msg->detections.size()==0){
      return;
    }
    ROS_INFO("get yolo data ....");
    cv_bridge::CvImagePtr cv_ptr;
    
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3 );
      original_image = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
   
    try
    {
      transformStamped = tfBuffer.lookupTransform(camera_tf, map_tf, ros::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      return;
    }

    tf2::Transform transform_tfmsg;
    transformMsgToTF2(transformStamped.transform, transform_tfmsg);



    tf2::Vector3 map_point=transform_tfmsg.getOrigin();
    tf2::Vector3 map_z(0,0,1);
    tf2::Vector3 normal_vector=transform_tfmsg * map_z - map_point;
       
    plane_.header.frame_id = camera_tf;
    plane_.header.stamp = ros::Time::now();

    
    plane_input.header.frame_id = camera_tf;
    plane_input.header.stamp = ros::Time::now();

    plane_.coefficients[0]=normal_vector.getX();
    plane_.coefficients[1]=normal_vector.getY();
    plane_.coefficients[2]=normal_vector.getZ();
    plane_.coefficients[3]=-normal_vector.getX()*map_point.getX()-normal_vector.getY()*map_point.getY()-normal_vector.getZ()*map_point.getZ();
    plane_.resolution=10;


    writing_file.clear();

    std::ostringstream oss;
    oss << file_numbering;

    std::string file = write_file_path + "/" + oss.str() ;
    file_csv = file + ".csv";
    std::string file_png = file + ".png";
    cv::imwrite(file_png,original_image); //ok????    

    
    for(int i=0;i<yolo_msg->detections.size();i++){
      col_px = yolo_msg->detections[i].bbox.center.x;
      row_px = yolo_msg->detections[i].bbox.center.y;
      image_to_tf::set_line(cv::Point(col_px, row_px));

      float intersection_z= -plane_.coefficients[3]/(normal_vector.getX()*c_x+normal_vector.getY()*c_y+normal_vector.getZ());
      float intersection_x = intersection_z*c_x;
      float intersection_y = intersection_z*c_y;
      
      geometry_msgs::Point point_msg1;
      point_msg1.x=0;
      point_msg1.y=0;
      point_msg1.z=0;
      plane_.cells.push_back(point_msg1);

      plane_input.grids.push_back(plane_);

      //plane_pub.publish(plane_input);

      tf2::Vector3 intersection(intersection_x,intersection_y,intersection_z);
      tf2::Transform child_to_parent =transform_tfmsg.inverse();
      tf2::Vector3 line_mapping= child_to_parent * intersection;
      

      

      visualization_msgs::Marker line_strip_plane;
      line_strip_plane.header.frame_id = map_tf;
      line_strip_plane.header.stamp = ros::Time::now();
      line_strip_plane.ns = "points_and_lines";
      line_strip_plane.action = visualization_msgs::Marker::ADD;
      line_strip_plane.pose.orientation.w = 1.0;

      line_strip_plane.id = 0;
      line_strip_plane.type = visualization_msgs::Marker::LINE_STRIP;

      // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
      line_strip_plane.scale.x = 0.1;
      // Line list is red
      line_strip_plane.color.r = 1.0;
      line_strip_plane.color.a = 1.0;

      
      line_strip_high.header.frame_id = "drone_xy";
      line_strip_high.header.stamp = ros::Time::now();
      line_strip_high.ns = "high";
      line_strip_high.action = visualization_msgs::Marker::ADD;
      line_strip_high.pose.orientation.w = 1.0;

      line_strip_high.id = 0;
      line_strip_high.type = visualization_msgs::Marker::LINE_STRIP;

      // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
      line_strip_high.scale.x = 0.1;
      // Line list is red
      line_strip_high.color.r = 0.0;
      line_strip_high.color.a = 1.0;
      line_strip_high.points.clear();




      // Create the vertices for the points and lines
      
      geometry_msgs::Point p_1;
      p_1.x = child_to_parent.getOrigin().getX();
      p_1.y = child_to_parent.getOrigin().getY();
      p_1.z = 0;
      line_strip_plane.points.push_back(p_1);
      p_1.z = child_to_parent.getOrigin().getZ();

      geometry_msgs::Point p_2;
      p_2.x = line_mapping.getX();
      p_2.y = line_mapping.getY();
      p_2.z = line_mapping.getZ();
      line_strip_plane.points.push_back(p_2);

        

      marker_pub_p.publish(line_strip_plane);

      
      get_line_high(p_1,p_2);

      geometry_msgs::TransformStamped transformStamped2;
      transformStamped2.header.stamp = ros::Time::now();
      transformStamped2.header.frame_id = map_tf;
      transformStamped2.child_frame_id = "drone_xy";
      transformStamped2.transform.translation.x = p_1.x;
      transformStamped2.transform.translation.y = p_1.y;
      transformStamped2.transform.translation.z = 0;
      transformStamped2.transform.rotation.x = 0;
      transformStamped2.transform.rotation.y = 0;
      transformStamped2.transform.rotation.z = 0;
      transformStamped2.transform.rotation.w = 1;
      dynamic_br_drone.sendTransform(transformStamped2);

      transformMsgToTF2(transformStamped2.transform, nav_to_dronexy);

      
      
      geometry_msgs::TransformStamped transformStamped;
      transformStamped.header.stamp = ros::Time::now();
      transformStamped.header.frame_id = camera_tf;
      transformStamped.child_frame_id = "target";
      transformStamped.transform.translation.x = intersection_x;
      transformStamped.transform.translation.y = intersection_y;
      transformStamped.transform.translation.z = intersection_z;
      transformStamped.transform.rotation.x = 0;
      transformStamped.transform.rotation.y = 0;
      transformStamped.transform.rotation.z = 0;
      transformStamped.transform.rotation.w = 1;
      dynamic_br_.sendTransform(transformStamped);

      tf2::Vector3 intersection_tf(intersection_x,intersection_y,intersection_z); 
      float intersection_Lon,intersection_Lat;
      tf_to_LonLat_simple(intersection_tf * magnification , intersection_Lon , intersection_Lat);
      //ROS_INFO("tf1 intersection_Lat:%f intersection_Lat:%f ", intersection_Lat,intersection_Lat);
      
      }
  writing_file.close();
  file_numbering++;
  std_msgs::String file_msg;
  file_msg.data=file;
  save_file_pub.publish(file_msg);


  }




void image_to_tf::get_line_high(const geometry_msgs::Point& p_P, geometry_msgs::Point& p_C)
  {   
    float dx = p_C.x- p_P.x ;
    float dy = p_C.y- p_P.y ;
    float dz = std::abs(p_C.z - p_P.z) ;

    ROS_INFO("p_C.z:%f p_P.z:%f ",  p_C.z, p_P.z);
    float before_error_hiegh=0;
    
    
    if (p_P.x>p_C.x){
      for (float start_i=p_P.x; start_i > p_C.x; start_i-=0.02 ){
        float line_x = start_i;
        float line_y = (dy) /(dx) * (start_i - p_P.x) + p_P.y;
        float d_L =sqrtf((line_x- p_P.x)*(line_x- p_P.x)  + (line_y- p_P.y)*(line_y- p_P.y)) ;

        tf2::Vector3 line_xy(line_x,line_y,0);
        float line_lat,line_lon;
        tf_to_LonLat_simple(line_xy,line_lon,line_lat);

        int i =  ( line_lon - head[0]  ) * head[4] ;
        int j =  ( line_lat - head[1]  ) * head[4] ;

        //if (i > head[2] || j > head[3]){
        if (i > 999 || j > 999){
            ROS_WARN("invalid lon or lat . something is wrong");
            break;
        }

        if (i < 0 || j < 0){
            ROS_WARN("invalid lon or lat minus. something is wrong");
            break;
        }

        ROS_INFO("line_lat - head[1]:%f line_lon - head[0]:%f :head%f",  line_lat - head[1], line_lon - head[0],head[4]);
        ROS_INFO("line_x:%f line_y:%f ",  line_x, line_y);
        ROS_INFO("line_lat:%f line_lon:%f ",  line_lat, line_lon);
        ROS_INFO("i:%d j:%d ",  i , j );
        ROS_INFO("Score:%f ",  Score[i][j]);

        float error_hiegh = dz-dz /sqrtf(dx*dx+dy*dy) * d_L - (Score[i][j] -start_high)/magnification;

        ROS_INFO("p_P.x>p_C.x  high drone %f; high lined offset %f ; target point high %f ;magn target point high %f",
                       dz , d_L,Score[i][j] - start_high ,(Score[i][j] -start_high)/magnification);
        
        geometry_msgs::Point p_h;
        p_h.x = line_x- p_P.x;
        p_h.y = line_y- p_P.y;
        p_h.z = (Score[i][j] -start_high)/magnification;

        line_strip_high.points.push_back(p_h);
        

        /*if (std::abs(error_hiegh) < thresh_humandetection){
          ROS_INFO("detect human position %f ",  error_hiegh );
          geometry_msgs::TransformStamped transformStamped;
          transformStamped.header.stamp = ros::Time::now();
          transformStamped.header.frame_id = "drone_xy";
          transformStamped.child_frame_id = "target_highfix";
          transformStamped.transform.translation.x = line_x- p_P.x;
          transformStamped.transform.translation.y = line_y- p_P.y;
          transformStamped.transform.translation.z = dz-dz /sqrtf(dx*dx+dy*dy) * d_L;
          transformStamped.transform.rotation.x = 0;
          transformStamped.transform.rotation.y = 0;
          transformStamped.transform.rotation.z = 0;
          transformStamped.transform.rotation.w = 1;
          dynamic_high_.sendTransform(transformStamped);
          marker_pub_h.publish(line_strip_high);

          //break;
        } */

        if (before_error_hiegh < 0 ){
          ROS_INFO("detect human position ; need another problem to improve accuracy %f ",  error_hiegh );
          geometry_msgs::TransformStamped transformStamped;
          transformStamped.header.stamp = ros::Time::now();
          transformStamped.header.frame_id = "drone_xy";
          transformStamped.child_frame_id = "target_highfix";
          transformStamped.transform.translation.x = line_x- p_P.x;
          transformStamped.transform.translation.y = line_y- p_P.y;
          transformStamped.transform.translation.z = dz-dz /sqrtf(dx*dx+dy*dy) * d_L;
          transformStamped.transform.rotation.x = 0;
          transformStamped.transform.rotation.y = 0;
          transformStamped.transform.rotation.z = 0;
          transformStamped.transform.rotation.w = 1;
          dynamic_high_.sendTransform(transformStamped);
          marker_pub_h.publish(line_strip_high);

          tf2::Vector3 drone_to_target(line_x- p_P.x,line_y- p_P.y,dz-dz /sqrtf(dx*dx+dy*dy) * d_L);
          tf2::Vector3 origin_to_target = nav_to_dronexy.inverse() * drone_to_target;

          file_writter(origin_to_target);

          break;
        }
        else{
          before_error_hiegh = error_hiegh;
        }

        marker_pub_h.publish(line_strip_high);

      }
    }else{
      for (float start_i=p_P.x; start_i < p_C.x; start_i+=0.1 ){
        float line_x = start_i;
        float line_y = (dy) /(dx) * (start_i - p_P.x) + p_P.y;
        float d_L =sqrtf((line_x- p_P.x)*(line_x- p_P.x)  + (line_y- p_P.y)*(line_y- p_P.y)) ;

        tf2::Vector3 line_xy(line_x,line_y,0);
        float line_lat,line_lon;
        tf_to_LonLat_simple(line_xy * magnification,line_lon,line_lat);

        int i =  ( line_lon - head[0]  ) * head[4] ;
        int j =  ( line_lat - head[1]  ) * head[4] ;

        //if (i > head[2] || j > head[3]){
        if (i > 999 || j > 999){
            ROS_WARN("invalid lon or lat . something is wrong");
            break;
        }
        if (i < 0 || j < 0){
            ROS_WARN("invalid lon or lat minus. something is wrong");
            break;
        }

        ROS_INFO("head[0]:%f  head[1]:%f :head[2]%f :head[3]%f :head[4]%f",  head[0], head[1], head[2], head[3],head[4]);
        ROS_INFO("line_x:%f line_y:%f ",  line_x, line_y);
        ROS_INFO("line_lat:%f line_lon:%f ",  line_lat, line_lon);
        ROS_INFO("i:%d j:%d Score:%f ",  i , j  ,  Score[i][j]);

        float error_hiegh = dz-dz /sqrtf(dx*dx+dy*dy) * d_L -(Score[i][j] -start_high )/magnification;
        ROS_INFO("p_P.x>p_C.x  high drone %f; high lined offset %f ; target point high %f ;magin target point high %f",  dz , d_L,Score[i][j] -start_high ,(Score[i][j] -start_high)/magnification);

        geometry_msgs::Point p_h;
        p_h.x = line_x- p_P.x;
        p_h.y = line_y- p_P.y;
        p_h.z = (Score[i][j] -start_high)/magnification;

        line_strip_high.points.push_back(p_h);

        /*if (std::abs(error_hiegh) < thresh_humandetection){
          ROS_INFO("detect human position %f ",  error_hiegh );
          geometry_msgs::TransformStamped transformStamped;
          transformStamped.header.stamp = ros::Time::now();
          transformStamped.header.frame_id = "drone_xy";
          transformStamped.child_frame_id = "target_highfix";
          transformStamped.transform.translation.x = line_x- p_P.x;
          transformStamped.transform.translation.y = line_y- p_P.y;
          transformStamped.transform.translation.z = dz-dz /sqrtf(dx*dx+dy*dy) * d_L;
          transformStamped.transform.rotation.x = 0;
          transformStamped.transform.rotation.y = 0;
          transformStamped.transform.rotation.z = 0;
          transformStamped.transform.rotation.w = 1;
          dynamic_high_.sendTransform(transformStamped);
          marker_pub_h.publish(line_strip_high);

          //break;
        }*/

        if (before_error_hiegh < 0 ){
          ROS_INFO("detect human position ; need another problem to improve accuracy %f ",  error_hiegh );
          geometry_msgs::TransformStamped transformStamped;
          transformStamped.header.stamp = ros::Time::now();
          transformStamped.header.frame_id = "drone_xy";
          transformStamped.child_frame_id = "target_highfix";
          transformStamped.transform.translation.x = line_x- p_P.x;
          transformStamped.transform.translation.y = line_y- p_P.y;
          transformStamped.transform.translation.z = dz-dz /sqrtf(dx*dx+dy*dy) * d_L;
          transformStamped.transform.rotation.x = 0;
          transformStamped.transform.rotation.y = 0;
          transformStamped.transform.rotation.z = 0;
          transformStamped.transform.rotation.w = 1;
          dynamic_high_.sendTransform(transformStamped);
          marker_pub_h.publish(line_strip_high);

          tf2::Vector3 drone_to_target(line_x- p_P.x,line_y- p_P.y,dz-dz /sqrtf(dx*dx+dy*dy) * d_L);
          tf2::Vector3 origin_to_target = nav_to_dronexy.inverse() * drone_to_target;

          file_writter(origin_to_target);

          break;
        }
        else{
          before_error_hiegh = error_hiegh;
        }
        marker_pub_h.publish(line_strip_high);
     
      }
    }
  }

void image_to_tf::transformMsgToTF2(const geometry_msgs::Transform& msg, tf2::Transform& tf2)
{tf2 = tf2::Transform(tf2::Quaternion(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w), tf2::Vector3(msg.translation.x, msg.translation.y, msg.translation.z));}

void image_to_tf::text_reader(const std::string& file_path)
  {
    std::ifstream ifs;
    ifs.open(file_path.c_str());   
    
    //float Score[100][100];
    
    std::string str = "";

    int i= -1;
    int j =0;
    while(getline(ifs, str))
    {
        std::string tmp ;
        std::istringstream stream(str);
 
        while (getline(stream, tmp, ' '))
        {
            float tmp_f ;
            std::istringstream(tmp) >> tmp_f;

            if (i==-1) {
              head[j] = tmp_f;
              j++;
            }
            else{
              Score[i][j] = tmp_f;
              j++;
            }
        }
        i++;
        j = 0;
    }
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
    float z = Att-magin_high  ;
    output_tf.setValue(x,y,z); 
  }


void image_to_tf::set_line(const cv::Point& point)
  {
     
    c_x=(point.x-camera_cx)/camera_fx;
    c_y=(point.y-camera_cy)/camera_fy;

    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = frame_id;
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "points_and_lines";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;

    line_strip.id = 0;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;
    // Line list is red
    line_strip.color.r = 1.0;
    line_strip.color.a = 1.0;

    // Create the vertices for the points and lines
    for (uint32_t i = 0; i < 10; ++i)
    { 
      geometry_msgs::Point p;
      p.x = c_x * i;
      p.y = c_y * i;
      p.z = i;

      line_strip.points.push_back(p);

    }

    marker_pub.publish(line_strip);
  }
void image_to_tf::file_writter(const tf2::Vector3& target){
  ROS_INFO("writting.....");
  float target_lon,target_lat;

  tf_to_LonLat_simple(target,target_lon,target_lat);

  if (writing_file.is_open()){
      writing_file << std::fixed <<std::setprecision(15) << "target_x:"<< target.getX() << ",target_y"<< target.getY() << ",targetz:" << target.getZ()
               << ",target_lon:" << target_lon  <<",target_lat:" << target_lat  
               << ",drone_lon:" << gps_lon  <<",drone_lat:" << gps_lat <<std::endl; 
      
  }else{
      writing_file.open(file_csv.c_str());
      writing_file << std::fixed <<std::setprecision(15) << "target_x:"<< target.getX() << ",target_y"<< target.getY() << ",targetz:" << target.getZ()
               << ",target_lon:" << target_lon  <<",target_lat:" << target_lat 
               << ",drone_lon:" << gps_lon  <<",drone_lat:" << gps_lat<<  std::endl; 
  }
  
  
 
  

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
