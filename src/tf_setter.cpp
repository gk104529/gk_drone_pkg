#include <ros/ros.h>
#include <cstdio>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <turtlesim/Spawn.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


class tf_setter
{
public:
  tf_setter();

private:
  void tf_broadcaster(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& input_pose);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh;
  ros::Subscriber init_sub;
  std::string parent_tf,child_tf;
  tf2_ros::TransformBroadcaster dynamic_br_;
};


tf_setter::tf_setter():
  pnh("~")
{

  pnh.param("parent_tf", parent_tf,std::string("odom"));
  pnh.param("child_tf", child_tf,std::string("base_link"));

  geometry_msgs::TransformStamped transformStamped;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  try
  {
    transformStamped = tfBuffer.lookupTransform(parent_tf, child_tf, ros::Time(10));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }

 
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
