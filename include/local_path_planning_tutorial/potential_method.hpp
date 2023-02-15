#ifndef POTENTIAL_METHOD
#define POTENTIAL_METHOD

#include <ros/ros.h>
#include <vector>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

typedef struct {         
    double x; 
    double y; 
    double potential;
    double org_potential;
} PotentialField;

namespace local_path_planning_tutorial {
    class PotentialMethod {
        private :
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            ros::Publisher pub_path_marker_;
            ros::Publisher pub_field_;
            double weight_obs_;
            double weight_goal_;
            double delta_;
            double grid_width_;

            void displayOptimalPathMarker ( double optimal_vel, double optimal_ang_vel );
            void dispPotentialField( const std::vector<PotentialField>& pot_field, const double min_m, const double max_m, const double min_p, const double max_p );
            
            double calPotential( const geometry_msgs::Point& base, const geometry_msgs::Point& target, const bool negative );
            
            double calPotential( const double x, const double y, const geometry_msgs::Point& target, const bool negative );
            
            bool calPotentialField( 
                const geometry_msgs::Point& target,
                const PointCloud::Ptr obstacles, 
                std::vector<PotentialField>* pot_field );
            
            bool calPath(
                const geometry_msgs::Point& target,
                const PointCloud::Ptr obstacles,
                geometry_msgs::TwistPtr path );

        public :
            PotentialMethod(  );
            bool potentialField ( 
                const geometry_msgs::Point& target,
                const PointCloud::Ptr obstacles );

            bool generatePath2Target(  
                const geometry_msgs::Point& target,
                const PointCloud::Ptr obstacles,
                geometry_msgs::TwistPtr path );

            void setWeightObstacle( const double weight );
            void setWeightGoal( const double weight );
            void setDelta( const double delta );
            void setGridWidth( const double grid_width );
    };  
}

inline double local_path_planning_tutorial::PotentialMethod::calPotential( const geometry_msgs::Point& base, const geometry_msgs::Point& target, const bool negative ) {
    double pot = ( 1 / ( 1 +  std::hypotf(base.x - target.x, base.y - target.y) ) );
    return (negative) ? -pot : pot; 
}
inline double local_path_planning_tutorial::PotentialMethod::calPotential( const double x, const double y, const geometry_msgs::Point& target, const bool negative ) {
    double pot = ( 1 / ( 1 +  std::hypotf(x - target.x, y - target.y) ) );
    return (negative) ? -pot : pot; 
}
inline void local_path_planning_tutorial::PotentialMethod::setWeightObstacle( const double weight ) { weight_obs_ = weight; }
inline void local_path_planning_tutorial::PotentialMethod::setWeightGoal( const double weight ) { weight_goal_ = weight; }
inline void local_path_planning_tutorial::PotentialMethod::setDelta( const double delta ) { delta_ = delta; }
inline void local_path_planning_tutorial::PotentialMethod::setGridWidth( const double grid_width ) { grid_width_ = grid_width; }

#endif