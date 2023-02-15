#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

typedef struct {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
}RobotPose;

RobotPose g_robot;
nav_msgs::Odometry g_odom;

void callbackTwist ( const geometry_msgs::TwistConstPtr &msg ) {
    g_robot.theta += 0.05 * msg->angular.z ;
    if ( g_robot.theta > M_PI )     g_robot.theta = g_robot.theta - 2 * M_PI;
    if ( g_robot.theta < - M_PI )   g_robot.theta = g_robot.theta + 2 * M_PI;

    g_robot.x += 0.05 * msg->linear.x * std::cos( g_robot.theta );
    g_robot.y += 0.05 * msg->linear.x * std::sin( g_robot.theta );

    g_odom.twist.twist.linear.x += 0.1 * ( msg->linear.x - g_odom.twist.twist.linear.x );
    g_odom.twist.twist.angular.z += 0.1 * ( msg->angular.z - g_odom.twist.twist.angular.z );
    g_odom.pose.pose.position.x = g_robot.x;
    g_odom.pose.pose.position.y = g_robot.y;
    tf::Quaternion q;
    q.setRPY(0, 0, g_robot.theta);
    g_odom.pose.pose.orientation.w = q.getW();
    g_odom.pose.pose.orientation.x = q.getX();
    g_odom.pose.pose.orientation.y = q.getY();
    g_odom.pose.pose.orientation.z = q.getZ();
    return;
}


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "robot_pose_broadcaster_node");

	ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ros::Subscriber sub_robot_pose = nh.subscribe( pnh.param<std::string>( "teleop_topic_name", "/cmd_vel_mux/input/teleop" ), 10, &callbackTwist );
    ros::Publisher pub_odom = nh.advertise< nav_msgs::Odometry >( pnh.param<std::string>( "odom_topic_name", "/odom" ), 1 );
    std::string base_footprint_name = pnh.param<std::string>( "base_footprint_name", "base_footprint" );
    static tf::TransformBroadcaster br;

    g_robot.x = 0.0;
    g_robot.y = 0.0;
    g_robot.theta = 0.0;

    ros::Rate rate(50);
    while(ros::ok()){
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(g_robot.x, g_robot.y, 0.0) );
        tf::Quaternion q;
        q.setRPY(0, 0, g_robot.theta);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", base_footprint_name ));
        pub_odom.publish( g_odom );
        ROS_INFO("[ Broadcaster ] x = %.3f , y = %.3f", g_robot.x, g_robot.y );
        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();
}