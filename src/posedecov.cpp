#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

ros::Publisher pub;

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped msg)
{
	geometry_msgs::Pose p;
	
	p.position.x = msg.pose.pose.position.x;
	p.position.y = msg.pose.pose.position.y;
	p.position.z = msg.pose.pose.position.z;

	p.orientation.x = msg.pose.pose.orientation.x;
	p.orientation.y = msg.pose.pose.orientation.y;
	p.orientation.z = msg.pose.pose.orientation.z;
	p.orientation.w = msg.pose.pose.orientation.w;

	pub.publish(p);
}

int main(int argc , char ** argv)
{
	ros::init(argc , argv , "posedecov");
	ros::NodeHandle n;
	pub = n.advertise<geometry_msgs::Pose>("/posedecov" , 1);
	ros::Subscriber sub = n.subscribe("/estimated_pose" , 1000 , poseCallback);
	ros::spin();
	return 0;
}
