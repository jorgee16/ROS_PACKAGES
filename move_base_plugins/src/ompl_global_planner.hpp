#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include <tf/tf.h>
#include <pluginlib/class_list_macros.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>

#include <iostream>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>


namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace move_base_plugins{

class OMPLGlobalPlanner : public nav_core::BaseGlobalPlanner {
public:
	
	// ROS move_base plugin
	OMPLGlobalPlanner(){}
	OMPLGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
		initialize(name, costmap_ros);
	};

	void initialize(std::string planner_name, costmap_2d::Costmap2DROS* ros_costmap);
	bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& gplan);
	~OMPLGlobalPlanner(){}

	// OMPL
	bool isStateValid(const ob::State *state);
	void plan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& gplan);
	void displayPath(const og::PathGeometric& path);
	void displayRRT(const ob::PlannerData& pd);

private:
	costmap_2d::Costmap2D* m_costmap;
	base_local_planner::WorldModel* m_world;
	double m_resolution;
	double m_dist_from_robot;
	ros::Publisher m_publ_path;
	ros::Publisher m_publ_rrt;
    double m_startx;
    double m_starty;

};

}
