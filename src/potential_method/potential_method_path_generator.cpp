#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include<local_path_planning_tutorial/potential_method.hpp>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class PotentialMethodPathGenerator {
    private :
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Publisher pub_cmd_vel_;
        ros::Publisher pub_path_marker_;
        
        ros::Subscriber sub_odom_;
        ros::Subscriber sub_sensor_;
        ros::Subscriber sub_tgt_pose_;
        
        tf::TransformListener tf_listener_;
        std::string base_footprint_name_;

        geometry_msgs::Twist curt_vel_;
        geometry_msgs::Point rob_pt_map_;
        geometry_msgs::Point tgt_pt_map_;
        geometry_msgs::Point tgt_pt_base_;
        PointCloud::Ptr obs_map_;

        visualization_msgs::Marker path_mrk_;
        local_path_planning_tutorial::PotentialMethod pot_;

        bool is_target_;

        geometry_msgs::Point transformPoint ( std::string org_frame, std::string target_frame, geometry_msgs::Point point );
        void callbackOdometry ( const nav_msgs::OdometryConstPtr &odom_msg );
        void callbackSensor( const sensor_msgs::PointCloud2ConstPtr &sensor_msg );
        void callbackTarget ( const geometry_msgs::PoseStampedConstPtr &msg );

    public:
        PotentialMethodPathGenerator();
};

geometry_msgs::Point PotentialMethodPathGenerator::transformPoint ( std::string org_frame, std::string target_frame, geometry_msgs::Point point ) {
    geometry_msgs::PointStamped pt_transformed;
    geometry_msgs::PointStamped pt;
    pt.header.frame_id = org_frame;
    pt.header.stamp = ros::Time(0);
    pt.point = point;
    if ( tf_listener_.frameExists( target_frame ) ) {
        try {tf_listener_.transformPoint( target_frame, pt, pt_transformed );}
        catch (tf::TransformException ex ){ROS_ERROR( "%s",ex.what( ) );}
    } else ROS_ERROR("target_frame is not Exists");
    return pt_transformed.point;
}

void PotentialMethodPathGenerator::callbackOdometry ( const nav_msgs::OdometryConstPtr &odom_msg ) { 
    curt_vel_ = odom_msg->twist.twist; 
    rob_pt_map_ = odom_msg->pose.pose.position;
    return;
}

void PotentialMethodPathGenerator::callbackSensor( const sensor_msgs::PointCloud2ConstPtr &sensor_msg ) {
    PointCloud::Ptr cloud_src (new PointCloud());
    PointCloud::Ptr cloud_obs (new PointCloud());
    pcl::fromROSMsg<PointT>( *sensor_msg, *cloud_src );
    *obs_map_ = *cloud_src;
    obs_map_->header.frame_id = "map";
    try {
        tf_listener_.waitForTransform( base_footprint_name_, "map", ros::Time(0), ros::Duration(1.0));
        pcl_ros::transformPointCloud( base_footprint_name_, ros::Time(0), *cloud_src, "map",  *cloud_obs, tf_listener_);
    } catch (tf::TransformException ex) {ROS_ERROR("%s", ex.what());}
    tgt_pt_base_ = transformPoint( "map",  base_footprint_name_, tgt_pt_map_ );

    geometry_msgs::TwistPtr path ( new geometry_msgs::Twist );
    double dist = std::hypotf( tgt_pt_base_.x, tgt_pt_base_.y );
    double ang = std::atan2( tgt_pt_base_.y, tgt_pt_base_.x );
    if( !is_target_ || dist < 0.1 ) {
        is_target_ = false;
        return;
    }

    pot_.generatePath2Target( tgt_pt_base_, cloud_obs, path );

    std::cout << "\n========================================\n[ Pos ]"
        << "\nrobot_map  = ( " << rob_pt_map_.x << ", " << rob_pt_map_.y << " )"
        << "\ntarget_map = ( " << tgt_pt_map_.x << ", " << tgt_pt_map_.y << " )"
        << "\ntarget_base = ( " << tgt_pt_base_.x << ", " << tgt_pt_base_.y << " )"
        << "\n\n[info]" 
        << "\n dist = " << dist
        << "\n ang = " << ang
        << "\n\n[ Path ]"
        << "\nlin = " << path->linear.x
        << "\nang = " << path->angular.z  
        << "\n========================================\n" 
    << std::endl;

    geometry_msgs::Point temp;
    temp.x = rob_pt_map_.x;
    temp.y = rob_pt_map_.y;
    temp.z = 0.1;
    path_mrk_.points.push_back( temp );
    path_mrk_.header.stamp = ros::Time::now();
    pub_path_marker_.publish ( path_mrk_ );
    pub_cmd_vel_.publish( path );
    return;

    return;
}

void PotentialMethodPathGenerator::callbackTarget ( const geometry_msgs::PoseStampedConstPtr &msg ) {
    tgt_pt_map_ = msg->pose.position;
    path_mrk_.points.clear();
    pot_.potentialField( tgt_pt_map_, obs_map_ );
    is_target_ = true;
    return;
}

PotentialMethodPathGenerator::PotentialMethodPathGenerator() : nh_(), pnh_("~") {
    pub_cmd_vel_ = nh_.advertise< geometry_msgs::Twist >( pnh_.param<std::string>( "teleop_topic_name", "/cmd_vel_mux/input/teleop" ), 1 );
    pub_path_marker_ = nh_.advertise< visualization_msgs::Marker >( "trajectory_potential", 1 );
    tgt_pt_map_.x = 0.0;
    tgt_pt_map_.y = 0.0;

    base_footprint_name_ = pnh_.param<std::string>( "base_footprint_name",  "base_footprint" );

    sub_odom_ = nh_.subscribe( pnh_.param<std::string>( "odom_topic_name", "/odom" ), 1, &PotentialMethodPathGenerator::callbackOdometry, this );
    sub_sensor_ = nh_.subscribe( "/sensor", 1 , &PotentialMethodPathGenerator::callbackSensor, this );
    sub_tgt_pose_ = nh_.subscribe( "/move_base_simple/goal", 10, &PotentialMethodPathGenerator::callbackTarget, this );
    obs_map_.reset( new PointCloud() );

    path_mrk_.header.frame_id = "map";
    path_mrk_.header.stamp = ros::Time::now();
    path_mrk_.ns = "trajectory_potential";
    path_mrk_.id = 1;
    path_mrk_.type = visualization_msgs::Marker::LINE_STRIP;
    path_mrk_.action = visualization_msgs::Marker::ADD;
    //path_mrk_.lifetime = ros::Duration(1.0);
    path_mrk_.scale.x = 0.05;
    path_mrk_.color.a = 1.0;
    path_mrk_.color.r = 0.0;
    path_mrk_.color.g = 1.0;
    path_mrk_.color.b = 0.0;
    path_mrk_.pose.orientation.x = 0.0;
    path_mrk_.pose.orientation.y = 0.0;
    path_mrk_.pose.orientation.z = 0.0;
    path_mrk_.pose.orientation.w = 1.0;

    pot_.setWeightObstacle( pnh_.param<double>( "weight_obstacle", 1.0 ) );
    pot_.setWeightGoal( pnh_.param<double>( "weight_goal", 1.0 ) );
    pot_.setDelta( pnh_.param<double>( "delta", 0.1 ) );
    pot_.setGridWidth(  pnh_.param<double>( "grid_width", 0.25 ) );

    is_target_ = false;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "path_generator");
    PotentialMethodPathGenerator pg;
    ros::spin();
    return 0;
}