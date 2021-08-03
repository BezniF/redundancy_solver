#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(1000.0);
  while (node.ok()){
    transform.setOrigin( tf::Vector3(0.2, 0.0, 0.39) );
    transform.setRotation( tf::Quaternion(0, 0, 1, 0) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "world"));
    rate.sleep();
  }
  return 0;
};
