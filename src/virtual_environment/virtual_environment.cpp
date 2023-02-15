#include <ros/ros.h>
#include<bits/stdc++.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace ve{
    class VirtualEnvironment {
        private :
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            ros::Publisher pub_mrk_sensor_;
            ros::Publisher pub_mrk_tgt_;
            ros::Publisher pub_sensor_;
            ros::Subscriber sub_tgt_pose_;
            tf::TransformListener tf_listener_;
            PointCloud::Ptr cloud_sensor_map_;
            geometry_msgs::Point tgt_pt_;

            void callbackTarget ( const geometry_msgs::PoseStampedConstPtr &msg ) {
                tgt_pt_.x = msg->pose.position.x;
                tgt_pt_.y = msg->pose.position.y;
                return;
            }

            void makeVirtualEnvironment( int sensor_size ) {
                cloud_sensor_map_->header.frame_id = "map";
                cloud_sensor_map_->width  = sensor_size;
                cloud_sensor_map_->height = 1;
                cloud_sensor_map_->points.resize (cloud_sensor_map_->width * cloud_sensor_map_->height);
                std::random_device rnd;
                std::mt19937 mt(rnd()); 
                std::uniform_real_distribution<> rand5( -5.0, 5.0); 
                for ( auto& obs : cloud_sensor_map_->points ) {
                    obs.x = rand5(mt);
                    obs.y = rand5(mt);
                    obs.z = 0.0;
                    obs.x = ( std::fabs(obs.x) < 0.4 ) ? ( obs.x > 0 ) ? obs.x + 0.4 : obs.x - 0.4 : obs.x;
                    obs.y = ( std::fabs(obs.y) < 0.4 ) ? ( obs.y > 0 ) ? obs.y + 0.4 : obs.y - 0.4 : obs.y;
                }
                ROS_INFO("Make Virtual Environment... : sensor size = %d", sensor_size);
            }

            void displayTargetMarker (  ) {
                visualization_msgs::Marker marker;
                marker.header.frame_id = "map";
                marker.header.stamp = ros::Time::now();
                marker.ns = "target_pose";
                marker.id = 0;
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;

                marker.pose.position = tgt_pt_;

                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;

                marker.scale.x = 0.2;
                marker.scale.y = 0.2;
                marker.scale.z = 0.2;

                marker.color.r = 0.0f;
                marker.color.g = 0.0f;
                marker.color.b = 1.0f;
                marker.color.a = 1.0f;

                pub_mrk_tgt_.publish( marker );

                return;
            }

            void displaySensorMarker (  ) {
                int marker_num = 0;
                visualization_msgs::MarkerArray marker_array;
                for ( auto& pt : cloud_sensor_map_->points ) {
                    visualization_msgs::Marker obs;
                    obs.header.frame_id = "map";
                    obs.header.stamp = ros::Time::now();
                    obs.type = visualization_msgs::Marker::SPHERE;
                    obs.action = visualization_msgs::Marker::ADD;
                    obs.ns = "obstacles";
                    obs.id = marker_num;
                    obs.pose.position.x = pt.x;
                    obs.pose.position.y = pt.y;
                    obs.pose.position.z = pt.z;
                    obs.pose.orientation.x = 0.0;
                    obs.pose.orientation.y = 0.0;
                    obs.pose.orientation.z = 0.0;
                    obs.pose.orientation.w = 1.0;
                    obs.scale.x = 0.1;
                    obs.scale.y = 0.1;
                    obs.scale.z = 0.05;
                    obs.color.r = 0.0f;
                    obs.color.g = 1.0f;
                    obs.color.b = 0.0f;
                    obs.color.a = 1.0f;
                    marker_num++;
                    marker_array.markers.push_back( obs );

                    visualization_msgs::Marker cost;
                    cost.header.frame_id = "map";
                    cost.header.stamp = ros::Time::now();
                    cost.type = visualization_msgs::Marker::SPHERE;
                    cost.action = visualization_msgs::Marker::ADD;
                    cost.ns = "cost";
                    cost.id = marker_num;
                    cost.pose.position.x = pt.x;
                    cost.pose.position.y = pt.y;
                    cost.pose.position.z = pt.z;
                    cost.pose.orientation.x = 0.0;
                    cost.pose.orientation.y = 0.0;
                    cost.pose.orientation.z = 0.0;
                    cost.pose.orientation.w = 1.0;
                    cost.scale.x = 0.35;
                    cost.scale.y = 0.35;
                    cost.scale.z = 0.01;
                    cost.color.r = 1.0f;
                    cost.color.g = 0.0f;
                    cost.color.b = 0.0f;
                    cost.color.a = 1.0f;
                    marker_num++;
                    marker_array.markers.push_back( cost );
                }
                pub_mrk_sensor_.publish( marker_array );
            }

        public :
            VirtualEnvironment ( ){
                int sensor_size = pnh_.param<int>( "sensor_size", 50 );
                cloud_sensor_map_ .reset ( new PointCloud() );
                tgt_pt_.x = 0.0;
                tgt_pt_.y = 0.0;
                pub_mrk_tgt_ = nh_.advertise< visualization_msgs::Marker >( "/target_marker", 1 );
                pub_mrk_sensor_ = nh_.advertise< visualization_msgs::MarkerArray >( "/sensor_marker", 1 );
                pub_sensor_ = nh_.advertise< sensor_msgs::PointCloud2 >( "/sensor", 1 );
                sub_tgt_pose_ = nh_.subscribe( "/move_base_simple/goal", 10, &VirtualEnvironment::callbackTarget, this );
                makeVirtualEnvironment( sensor_size );
                displaySensorMarker();
                displayTargetMarker( );
            }

            void pubSensor (  ) {
                ros::Rate rate(30);
                while(ros::ok()){
                    PointCloud::Ptr cloud_sensor_base (new PointCloud() );
                    sensor_msgs::PointCloud2Ptr sensor_msg ( new sensor_msgs::PointCloud2 );
                    /*
                    try {
                        tf_listener_.waitForTransform("map", "map", ros::Time(0), ros::Duration(1.0));
                        pcl_ros::transformPointCloud("map", ros::Time(0), *cloud_sensor_map_, "map",  *cloud_sensor_base, tf_listener_);
                        cloud_sensor_base->header.frame_id = "map";
                    } catch (tf::TransformException ex) {
                        ROS_ERROR("%s", ex.what());
                        continue;
                    }
                    */
                    pcl::toROSMsg(*cloud_sensor_map_, *sensor_msg );
                    sensor_msg->header.stamp = ros::Time::now();
                    pub_sensor_.publish( sensor_msg );
                    displaySensorMarker ( );
                    displayTargetMarker( );
                    ROS_INFO("[ VirtualEnvironment ] publish sensor data !!");
                    ros::spinOnce();
                    rate.sleep();
                }
            }
    }; 
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "virtual_environment");
    ve::VirtualEnvironment ve;
    ve.pubSensor();
    ros::spin();
}