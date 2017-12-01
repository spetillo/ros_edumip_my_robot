#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
//#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <edumip_msgs/EduMipState.h>



class RobotJointPublisher
{
public:
  RobotJointPublisher();

private:
  void stateCallback(const edumip_msgs::EduMipState::ConstPtr& state);

  ros::NodeHandle nh_;

  ros::Publisher joint_pub_;
  ros::Subscriber state_sub_;

};



RobotJointPublisher::RobotJointPublisher()
{

  //PUBLISH: sensor_msgs/JointState topic named "/joint_states"
  joint_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);

  //SUBSCRIBE: edumip_msgs/EduMipState topic entitled "/edumip/state"
  state_sub_ = nh_.subscribe<edumip_msgs::EduMipState>("edumip/state", 10, &RobotJointPublisher::stateCallback, this);

}


void RobotJointPublisher::stateCallback(const edumip_msgs::EduMipState::ConstPtr& state)
{

  // robot state
  double angle = state->body_frame_heading;
  double tilt = state->theta;

  // create joint state 
  sensor_msgs::JointState joint_state;

  // update joint_state
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(2);
  joint_state.position.resize(2);
  joint_state.name[0] ="jointL";
  joint_state.position[0] = state->wheel_angle_L;
  joint_state.name[1] ="jointR";
  joint_state.position[1] = state->wheel_angle_R;


  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(state->body_frame_northing, -state->body_frame_easting, 0.0) );
  // transform.setOrigin( tf::Vector3(state->body_frame_northing, -state->body_frame_easting, 0.034) );
  tf::Quaternion q;
  q.setRPY(0, state->theta, -state->body_frame_heading);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "edumip_body"));

  
  // // create broadcaster
  // tf::TransformBroadcaster broadcaster;
  // geometry_msgs::TransformStamped odom_trans;
  // //tf::StampedTransform odom_trans;

  // odom_trans.header.frame_id = "odom";
  // odom_trans.child_frame_id = "axis";
  
  // // update transform
  // odom_trans.header.stamp = ros::Time::now();
  // odom_trans.transform.translation.x = state->body_frame_easting;
  // odom_trans.transform.translation.y = state->body_frame_northing;
  // odom_trans.transform.translation.z = 0;
  // odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);
  // // odom_trans.transform.translation.x = cos(angle)*2;
  // // odom_trans.transform.translation.y = sin(angle)*2;
  // // odom_trans.transform.translation.z = .7;
  // // odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

  //send the joint state and transform
  joint_pub_.publish(joint_state);
  // broadcaster.sendTransform(odom_trans);

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_joint_publisher");
  RobotJointPublisher pub_robot_joints;

  ros::spin();
}
