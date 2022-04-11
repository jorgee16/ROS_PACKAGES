#include "ompl_global_planner.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

PLUGINLIB_EXPORT_CLASS(move_base_plugins::OMPLGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace move_base_plugins{

bool OMPLGlobalPlanner::isStateValid(const ob::State *state){
	double x = state->as<ob::SE2StateSpace::StateType>()->getX();
	double y = state->as<ob::SE2StateSpace::StateType>()->getY();
	//state->as<ob::SE2StateSpace::StateType>()->getYaw();

	unsigned int cx=0, cy=0;
	double cost = -1.0;

    if (sqrt( (x - m_startx)*(x-m_startx) + (y - m_starty)*(y - m_starty)) < 0.30){ // m_startx?
        return true;
    }

	if (!m_costmap->worldToMap(x,y,cx,cy)){
		return false;
	}

	cost = m_costmap->getCost(cx,cy);

	if (cost < 10){
		return true;
	}
	else{
		return false;
	}
}

void OMPLGlobalPlanner::plan(const geometry_msgs::PoseStamped& startp, const geometry_msgs::PoseStamped& goalp, std::vector<geometry_msgs::PoseStamped>& gplan){

    m_startx = startp.pose.position.x;
    m_starty = startp.pose.position.y;
	// Create the search space
	std::shared_ptr<ob::SE2StateSpace> space(std::make_shared<ob::SE2StateSpace>());
	ob::RealVectorBounds bounds(2);

	bounds.setLow(0, -30);
	bounds.setHigh(0, +30);
	bounds.setLow(1, -30);
	bounds.setHigh(1, +30);

	space->setBounds(bounds);

	// 
	og::SimpleSetup ssetup(space);
	ssetup.setStateValidityChecker([this](const ob::State *state){return this->isStateValid(state);});

	//
	std::vector<double> vstart = {startp.pose.position.x,startp.pose.position.y, tf::getYaw(startp.pose.orientation)};
	ob::ScopedState<> start(space);
	start = vstart;

	std::vector<double> vgoal = {goalp.pose.position.x,goalp.pose.position.y, tf::getYaw(goalp.pose.orientation)};
	ob::ScopedState<> goal(space);
	goal = vgoal;

	ssetup.setStartAndGoalStates(start, goal);

    std::shared_ptr<og::RRTConnect> rrtc_ptr(std::make_shared<og::RRTConnect>(ssetup.getSpaceInformation()));
	rrtc_ptr->setRange(0.5);
	// rrtc_ptr->setGoalBias(0.5);
    // range, goal bias
	ssetup.setPlanner(rrtc_ptr);

    std::cerr << "Starting RRT solver: \n";
	ob::PlannerStatus solved = ssetup.solve(5.0);
	if (solved){

		std::cerr << "Solution found: \n";
		std::cerr << "Raw solution: \n";
		ssetup.getSolutionPath().print(std::cout);
		displayPath(ssetup.getSolutionPath());

		ob::PlannerData pd(ssetup.getSpaceInformation());
		ssetup.getPlannerData(pd);
		displayRRT(pd);

		ssetup.simplifySolution();
		std::cerr << "Simplified solution: \n";
		ssetup.getSolutionPath().print(std::cout);

		gplan.clear();
		for(int i=0; i< ssetup.getSolutionPath().getStateCount(); i++){
			auto st = ssetup.getSolutionPath().getState(i)->as<ob::SE2StateSpace::StateType>();	
			geometry_msgs::PoseStamped posest;
			posest.header.frame_id = "map";
			posest.header.stamp = ros::Time::now();
			posest.pose.position.x = st->getX();
			posest.pose.position.y = st->getY();
			posest.pose.position.z = 0;
			posest.pose.orientation.x = 0;
			posest.pose.orientation.y = 0;
			posest.pose.orientation.z = 0;
			posest.pose.orientation.w = 1;
			gplan.push_back(posest);

		}
	}
	else{
		ob::PlannerData pd(ssetup.getSpaceInformation());
		ssetup.getPlannerData(pd);
        displayRRT(pd);
		std::cout << "NO SOLUTION \n" ;
	}
}
void OMPLGlobalPlanner::displayRRT(const ob::PlannerData& pd){
	static int msg_id = 0;
	visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time::now();
	marker.id = msg_id++;
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.action = visualization_msgs::Marker::ADD;

	auto st = pd.getVertex(0).getState()->as<ob::SE2StateSpace::StateType>();
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0;
	marker.pose.orientation.y = 0;
	marker.pose.orientation.z = 0;
	marker.pose.orientation.w = 1;
	marker.scale.x = 0.02;
	//marker.scale.y = 0.1;
	//marker.scale.z = 0.1;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;

	std::cerr << "------------------------  " << pd.numVertices() << "  ------\n";
	for(int i=0; i<pd.numVertices(); i++){

		for(int j=0; j<pd.numVertices(); j++){

			if (pd.edgeExists(i, j)){
				auto st = pd.getVertex(i).getState()->as<ob::SE2StateSpace::StateType>();
				geometry_msgs::Point pt;
				pt.x = st->getX();
				pt.y = st->getY();
				pt.z = 0.0;
				marker.points.push_back(pt);

				auto sta = pd.getVertex(j).getState()->as<ob::SE2StateSpace::StateType>();
				geometry_msgs::Point pta;
				pta.x = sta->getX();
				pta.y = sta->getY();
				pta.z = 0.0;
				marker.points.push_back(pta);
			}
		}
	}

	m_publ_rrt.publish(marker);
}

void OMPLGlobalPlanner::displayPath(const og::PathGeometric& path){

	nav_msgs::Path pp;
	pp.header.frame_id = "map";
	pp.header.stamp = ros::Time();

	for(int i=0; i< path.getStateCount(); i++){
		auto st = path.getState(i)->as<ob::SE2StateSpace::StateType>();	
		geometry_msgs::PoseStamped posest;
		posest.header.stamp = ros::Time::now();
		posest.header.frame_id = "map";
		posest.pose.position.x = st->getX();
		posest.pose.position.y = st->getY();
		posest.pose.position.z = 0;
		posest.pose.orientation.x = 0;
		posest.pose.orientation.y = 0;
		posest.pose.orientation.z = 0;
		posest.pose.orientation.w = 1;

		pp.poses.push_back(posest);
	}
	m_publ_path.publish(pp);

}


void OMPLGlobalPlanner::initialize(std::string planner_name, costmap_2d::Costmap2DROS* ros_costmap){

	m_costmap = ros_costmap->getCostmap();

	ros::NodeHandle nh("~/" + planner_name);
	nh.param("step_size", m_resolution, m_costmap->getResolution());
	nh.param("min_dist_from_robot", m_dist_from_robot, 0.10);

	m_world = new base_local_planner::CostmapModel(*m_costmap);

	m_publ_path = nh.advertise<nav_msgs::Path>("/ompl/path", 1);
	m_publ_rrt = nh.advertise<visualization_msgs::Marker>("/ompl/rrt", 1);

}

bool OMPLGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& startp, const geometry_msgs::PoseStamped& goalp, std::vector<geometry_msgs::PoseStamped>& gplan){
	gplan.clear();

	this->plan(startp, goalp, gplan);

	return true;
}

}
