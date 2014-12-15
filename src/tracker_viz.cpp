#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

ros::Publisher marker_pub;

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped msg)
{
	visualization_msgs::Marker marker;
	uint32_t shape = visualization_msgs::Marker::CUBE;	
	marker.header.frame_id = "/my_frame";
	marker.header.stamp = ros::Time::now();

	marker.ns = "tracker_viz";
	marker.id = 0;

	//marker.type = shape;
	marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	marker.mesh_resource = "package://using_markers/pose_tracker3.stl";
	marker.action = visualization_msgs::Marker::ADD;
	
	marker.pose.position.x = msg.pose.pose.position.y * 10;
	marker.pose.position.y = -msg.pose.pose.position.x * 10;
	marker.pose.position.z = msg.pose.pose.position.z * 10;

	marker.pose.orientation.x = msg.pose.pose.orientation.y;
	marker.pose.orientation.y = -msg.pose.pose.orientation.x;
	marker.pose.orientation.z = msg.pose.pose.orientation.z;
	marker.pose.orientation.w = msg.pose.pose.orientation.w;

	marker.scale.x = 0.01;
	marker.scale.y = 0.01;
	marker.scale.z = 0.01;

	marker.color.r = 0.5f;
	marker.color.g = 0.5f;
	marker.color.b = 0.5f;
	marker.color.a = 0.5;

	marker.lifetime = ros::Duration();

	marker_pub.publish(marker);
}

int main(int argc , char ** argv)
{
	ros::init(argc , argv , "tracker_viz");
	ros::NodeHandle n;
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker" , 1);
	ros::Subscriber sub = n.subscribe("/estimated_pose" , 1000 , poseCallback);
	ros::spin();
	return 0;
}
