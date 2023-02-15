#include <local_path_planning_tutorial/potential_method.hpp>

using namespace local_path_planning_tutorial;

void PotentialMethod::displayOptimalPathMarker ( double optimal_vel, double optimal_ang_vel ) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_footprint";
    marker.header.stamp = ros::Time::now();
    marker.ns = "optimal_path";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.03;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(0.1);

    int predict_step = 30;
    double theta = optimal_ang_vel;
    double sampling_time = 0.1;
    geometry_msgs::Point pt, pre_pt;
    pt.z = 0.1;
    for ( int step = 0; step < predict_step; ++step) {
        pre_pt = pt;
        pt.x = optimal_vel * cos(theta) * sampling_time + pre_pt.x;
        pt.y = optimal_vel * sin(theta) * sampling_time + pre_pt.y;
        theta = optimal_ang_vel * sampling_time + theta;
        if ( std::hypotf( pt.x, pt.y ) > 1.5 ) break;
        marker.points.push_back( pt );
    }
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    pub_path_marker_.publish ( marker );
}


void PotentialMethod::dispPotentialField( const std::vector<PotentialField>& pot_field, const double min_m, const double max_m, const double min_p, const double max_p ) {
    visualization_msgs::MarkerArray pot_mrk;
    int marker_id = 0;
    double r0 = 1.0, g0 = 0.0, b0 = 0.0;
    double rh = 0.0, gh = 0.5, bh = 1.0;
    double h = 2.0;
    double grid_width = grid_width_;
    for ( auto& pf : pot_field ) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "potential_field";
        marker.id = marker_id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = pf.x;
        marker.pose.position.y = pf.y;
        marker.pose.position.z = pf.potential;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = grid_width;
        marker.scale.y = grid_width;
        marker.scale.z = 0.1;

        double pot = pf.org_potential + 1.0;

        marker.color.r = r0 + ( rh - r0 ) * pot/ h;
        marker.color.g = g0 + ( gh - g0 ) * pot/ h;
        marker.color.b = b0 + ( bh - b0 ) * pot/ h;
        marker.color.a = 0.4f;

        // marker.lifetime = ros::Duration(0.3);
        marker_id++;
        pot_mrk.markers.push_back( marker );
    }
    pub_field_.publish( pot_mrk );
    return;
}

bool PotentialMethod::calPotentialField(     
    const geometry_msgs::Point& target,
    const PointCloud::Ptr obstacles, 
    std::vector<PotentialField>* pot_field  ) 
{
    if( obstacles->points.size() == 0 ) return false;

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    double radius = 0.5;
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud (obstacles);
    pcl::PointXYZ search_pt;

    double weight_obs = weight_obs_;
    double weight_goal = weight_goal_;
    double grid_width = grid_width_;
    std::vector<PotentialField> pf;
    double min_m = DBL_MAX, max_m = DBL_MIN, min_p = DBL_MAX, max_p = DBL_MIN;

    for ( double x = -5.0; x < 5.0; x += grid_width ) {
        for ( double y = -5.0; y < 5.0; y += grid_width ) {
            // Searching for obstacles : ;
            double obs_pot = 0.0, goal_pot = 0.0;
            PotentialField tmp;
            geometry_msgs::Point obs_pt;
            search_pt.x = x;search_pt.y = y;search_pt.z = 0.0;
            if ( kdtree.radiusSearch (search_pt, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance, 1) > 0 ) {
                obs_pt.x = obstacles->points[ pointIdxRadiusSearch[0] ].x; 
                obs_pt.y = obstacles->points[ pointIdxRadiusSearch[0] ].y; 
                // Calculating Obstacle Potential :
                if ( std::hypotf( x - obs_pt.x, y - obs_pt.y ) < 0.05 ) obs_pot = 1.0;
                else obs_pot = calPotential( x, y, obs_pt, false );
            } 
            // Calculating Goal Potential :
            if( std::hypotf( x - target.x, y -target.y ) < 0.05 ) goal_pot = -1.0;
            else goal_pot = calPotential( x, y, target, true );
            tmp.potential = ( weight_obs * obs_pot ) + ( weight_goal * goal_pot );
            tmp.org_potential = obs_pot + goal_pot;
            tmp.x = x;
            tmp.y = y;
            pf.push_back( tmp );
            if ( tmp.potential < 0 ) {
                min_m = ( min_m > tmp.potential ) ? tmp.potential : min_m; 
                max_m = ( max_m < tmp.potential ) ? tmp.potential : max_m; 
            } else {
                min_p = ( min_p > tmp.potential ) ? tmp.potential : min_p; 
                max_p = ( max_m < tmp.potential ) ? tmp.potential : max_p; 
            }
        }
    }
    dispPotentialField( pf, min_m, max_m, min_p, max_p );
    *pot_field = pf;
    return true;
}

bool PotentialMethod::calPath(
    const geometry_msgs::Point& target,
    const PointCloud::Ptr obstacles,
    geometry_msgs::TwistPtr path ) 
{
    if( obstacles->points.size() == 0 ) return false;
    std::cout  << "\n============================================================"
            << "\n[Position]" 
            << "\n* x     : " << target.x
            << "\n* y     : " << target.y
    << std::endl;


    geometry_msgs::Twist tmp_path;

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    double radius = 0.5;
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud (obstacles);
    pcl::PointXYZ search_pt;

    double weight_obs = weight_obs_;
    double weight_goal = weight_goal_;
    double delta = delta_;
    double goal_dist = std::hypotf( target.x, target.y );

    geometry_msgs::Point obs_pt;
    search_pt.x = 0.0;search_pt.y = 0.0;search_pt.z = 0.0;
    double obs_pot = 0.0, obs_pot_x_del = 0.0, obs_pot_y_del = 0.0;
    double goal_pot = 0.0, goal_pot_x_del = 0.0, goal_pot_y_del = 0.0;

    
    // Calculating Obstacle Potential :
    if ( kdtree.radiusSearch (search_pt, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance, 1) > 0 ) {
        obs_pt.x = obstacles->points[ pointIdxRadiusSearch[0] ].x; 
        obs_pt.y = obstacles->points[ pointIdxRadiusSearch[0] ].y;
        if ( std::hypotf( obs_pt.x, obs_pt.y ) < 0.05 ) {
            obs_pot = 1.0;
            obs_pot_x_del = 1.0;
            obs_pot_y_del = 1.0;
        } else {
            obs_pot = calPotential( 0.0, 0.0, obs_pt, false );
            obs_pot_x_del = calPotential(delta, 0.0, obs_pt, false );
            obs_pot_y_del = calPotential( 0.0, delta, obs_pt, false );
        }
    }
    std::cout  << "\n[Obstacle Potential]" 
        << "\n* obs_pot         : " << obs_pot
        << "\n* obs_pot_x_del   : " << obs_pot_x_del
        << "\n* obs_pot_y_del   : " << obs_pot_y_del
    << std::endl;

    // Calculating Goal Potential :
    if(goal_dist < 0.05 ) {
        goal_pot = -1.0;
        goal_pot_x_del = -1.0; 
        goal_pot_y_del = -1.0;
    } else {
        goal_pot = calPotential( 0.0, 0.0, target, true );
        goal_pot_x_del = calPotential( delta, 0.0, target, true );
        goal_pot_y_del = calPotential( 0.0, delta, target, true );
    }
    std::cout  << "\n[Goal Potential]" 
        << "\n* goal_pot        : " << goal_pot
        << "\n* goal_pot_x_del  : " << goal_pot_x_del
        << "\n* goal_pot_y_del  : " << goal_pot_y_del
    << std::endl;

    double pot_xy = ( weight_obs * obs_pot ) + ( weight_goal * goal_pot ); 
    double pot_x_del = ( weight_obs * obs_pot_x_del ) + ( weight_goal * goal_pot_x_del ); 
    double pot_y_del = ( weight_obs * obs_pot_y_del ) + ( weight_goal * goal_pot_y_del );
    std::cout  << "\n[Total Potential]" 
        << "\n* pot_xy          : " << pot_xy
        << "\n* pot_x_del       : " << pot_x_del
        << "\n* pot_y_del       : " << pot_y_del
    << std::endl;

    // polarization of the potential field ( Calculate the gradient of the potential field ) :
    double vx = -( pot_x_del - pot_xy ) / delta;
    double vy = -( pot_y_del - pot_xy ) / delta;
    double v = std::hypotf( vx, vy );
    double vel =  1.3 / ( 1 + std::exp( -1.3 * 1.3 * ( goal_dist - 1 ) ) ); // Logistic function

    vx /= v / vel;
    vy /= v / vel;
    
    std::cout  << "\n[Polarization]" 
        << "\n* vx              : " << vx
        << "\n* vy              : " << vy
        << "\n* v               : " << v
        << "\n* vel             : " << vel
    << std::endl;

    geometry_msgs::Point next_pt;
    next_pt.x = vx;
    next_pt.y = vy;
    tmp_path.linear.x = std::hypotf( next_pt.x, next_pt.y );
    tmp_path.angular.z = std::atan2( next_pt.y, next_pt.x );
    std::cout << "\n[Velocity]"  
        << "\n* linear  [m/s]   : " <<  tmp_path.linear.x 
        << "\n* angular [rad/s] : " <<  tmp_path.angular.z 
    << std::endl;

    *path = tmp_path;
    return true;
}

PotentialMethod::PotentialMethod() : nh_(), pnh_("~") {
    pub_path_marker_ = nh_.advertise< visualization_msgs::Marker >( "/potential_path_marker", 1 );
    pub_field_ = nh_.advertise< visualization_msgs::MarkerArray >( "/potential_field", 1 );

    setWeightObstacle( pnh_.param<double>( "weight_obstacle", 1.0 ) );
    setWeightGoal( pnh_.param<double>( "weight_goal", 1.0 ) );
    setDelta( pnh_.param<double>( "delta", 0.1 ) );
    setGridWidth(  pnh_.param<double>( "grid_width", 0.25 ) );
    std::cout  << "\n============================================================"
        << "\n[ Potential Method Paramater]" 
        << "\n* weight_obstacle     : " << pnh_.param<double>( "weight_obstacle", 1.0 )
        << "\n* weight_goal         : " << pnh_.param<double>( "weight_goal", 1.0 )
        << "\n* delta               : " << pnh_.param<double>( "delta", 0.1 )
        << "\n* grid_width          : " << pnh_.param<double>( "grid_width", 0.25 )
    << std::endl;
}

bool PotentialMethod::potentialField( 
    const geometry_msgs::Point& target,
    const PointCloud::Ptr obstacles ) 
{
    std::vector<PotentialField> pot_field;
    calPotentialField( target, obstacles, &pot_field );
    return true;
}

bool PotentialMethod::generatePath2Target ( 
    const geometry_msgs::Point& target,
    const PointCloud::Ptr obstacles,
    geometry_msgs::TwistPtr path ) 
{   
    geometry_msgs::TwistPtr tmp_path ( new geometry_msgs::Twist );
    calPath( target, obstacles, tmp_path );
    *path = *tmp_path;
    displayOptimalPathMarker( tmp_path->linear.x, tmp_path->angular.z  );

    return true;
}


