
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

//#include <string>
//#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"	// sonars
#include "sensor_msgs/LaserScan.h"	// LRF
#include "geometry_msgs/Twist.h"	// cmd_vel

//parameters
double MAX_LINEAR_SPEED;
double MAX_ANGULAR_SPEED;
double SAFE_DIST_TH;
double DETOUR_OBST_TH;
double STOP_DIST_TH;
double DIFF_TH;
int verbose;
int useLRF;

//#define NSINVMAX 2

//topic to publish velocity commands
ros::Publisher cmd_vel_pub;

/*void sonarReceived(const sensor_msgs::Range::ConstPtr& pt){
	for (int i = 0; i < SONAR_ARRAY; ++i) ROS_INFO("Sonar %d: range=%f", i, pt->range);	
}*/

float Euclidist(float x, float y)
{ return sqrt(x*x+y*y); }

void sonarReceived(const sensor_msgs::PointCloud::ConstPtr& pt){
	int size = pt->points.size(); // number of points in the point cloud
	float *d = (float *) malloc(size * sizeof(float));
	int *a = (int *) malloc(size * sizeof(int));
	bool *f = (bool *) malloc(size * sizeof(bool));
	int i, idxMinDist = 0, ifront = 4;//size/2-1;
	static geometry_msgs::Twist cmd_vel;
	float w1, w2, wl, wr; // left weight (wl) and right weight (wr)
	static int nsinv; // number of consecutive signal inversions in angular velocity
	float w_ = cmd_vel.angular.z; // previous angular velocity
	static double eT;
	static bool recover = false; 
	const bool mask[16] = {false, true, true, true, true, true, true, false, false, false, false, false, false, false, false, false};
	
	if (recover && eT <= ros::Time::now().toSec())
	{
		recover = false; // finish recovery
		ROS_INFO("End of recovery rotation");
	}
	
	if (!recover) {
	// compute linear speed
	for (i = 0; i < size; i++)
	{
		//compute sensor angular position (+1 left, 0 front, -1 right, -2 behind)
		if (i < 15 && mask[i] && pt->points[i].x >= 0.0) // sensor in front
		{
			if (pt->points[i].y > 0.0) a[i] = 1;
			else if (pt->points[i].y < 0.0) a[i] = -1;
			else { a[i] = 0; ifront = i; }
		} else a[i] = -2;
		//compute distance and update minimum distance
		d[i] = Euclidist(pt->points[i].x, pt->points[i].y);
		if (a[i] > -2 && d[i] < d[idxMinDist]) idxMinDist = i;
		//compute flag (true if in front and nearer than SAFE_DIST_TH)
		if (a[i] > -2 && d[i] < SAFE_DIST_TH) f[i] = true; else f[i] = false; 
		if (verbose) ROS_INFO("%d : %f %d %d", i, d[i], a[i], f[i]);
	}
	if (!f[idxMinDist]) cmd_vel.linear.x = MAX_LINEAR_SPEED;
	else if (d[idxMinDist] > STOP_DIST_TH)
				cmd_vel.linear.x = MAX_LINEAR_SPEED *
							(d[idxMinDist] - STOP_DIST_TH) / (SAFE_DIST_TH - STOP_DIST_TH);
	else cmd_vel.linear.x = 0.0;

	// compute angular speed
	if (!f[idxMinDist]) cmd_vel.angular.z = 0.0;
	else
	{
		wl = 0.0; wr = 0.0;
		for (i = 0; i < size; i++)
			if (f[i])
			{
				w1 = (d[i]>0.0?fabs(pt->points[i].x)/d[i]:1.0); // weight as a function of absolute value of cos(angle): more lateral, less weight
				w2 = (d[i] <= DETOUR_OBST_TH ? 1.25 - d[i] / DETOUR_OBST_TH : 0.0);
				if (a[i] > 0) wl += w1 * w2;
				else if (a[i] < 0) wr += w1 * w2;
			}

		if ((wl > DIFF_TH || wr > DIFF_TH) && (wl > wr + DIFF_TH || wl + DIFF_TH < wr))
		{
			if (wl > wr + DIFF_TH ) // left obstacles have higher weight: rotate clockwise (towards negative angles)
				cmd_vel.angular.z = - MAX_ANGULAR_SPEED * wl;
			else if (wl + DIFF_TH < wr)  // right obstacles have higher weight: rotate anticlockwise (towards positive angles)
				cmd_vel.angular.z = MAX_ANGULAR_SPEED * wr;
		}
		else
		{
			if (verbose) ROS_INFO("Lateral readings have a negligible difference");
			//ROS_INFO("%d %d %d", size, ifront, idxMinDist);
			if (d[ifront] >= STOP_DIST_TH)
			{
				w1 = (d[ifront] - STOP_DIST_TH) / (SAFE_DIST_TH - STOP_DIST_TH);
				if (w1 < 0.25 ) w1 = 0.25;
				if (w1 > 1.0) w1 = 1.0;
			 	cmd_vel.linear.x = MAX_LINEAR_SPEED * w1;
				cmd_vel.angular.z = 0.0;
			}
			else
			{
				if (a[idxMinDist] >= 0) cmd_vel.angular.z = -MAX_ANGULAR_SPEED * 0.5;
				else cmd_vel.angular.z = MAX_ANGULAR_SPEED * 0.5;
				// check if there was a signal inversion in angular velocity
				if (cmd_vel.angular.z * w_ < 0.0) ++nsinv;
				if (nsinv > 1) {
					ROS_WARN(" ****** Deadlock Detected! ******");
					// keep current rotation speed until robot rotates PI
					eT = ros::Time::now().toSec() +
						3.14159/MAX_ANGULAR_SPEED * 2.0;
					recover = true;
					nsinv = 0;
				}
			}
		}
/*		if (wl > wr + DIFF_TH ) // left obstacles have higher weight: rotate clockwise (towards negative angles)
			cmd_vel.angular.z = - MAX_ANGULAR_SPEED * wl;
		else if (wl + DIFF_TH < wr)  // right obstacles have higher weight: rotate anticlockwise (towards positive angles)
			cmd_vel.angular.z = MAX_ANGULAR_SPEED * wr;
		else
		{
			if (verbose) ROS_INFO("Lateral readings have a negligible difference");
			if (d[ifront] >= STOP_DIST_TH)
			{
				cmd_vel.linear.x = MAX_LINEAR_SPEED *
							(d[ifront] - STOP_DIST_TH) / (SAFE_DIST_TH - STOP_DIST_TH);
				if (cmd_vel.linear.x > MAX_LINEAR_SPEED) cmd_vel.linear.x = MAX_LINEAR_SPEED;
				cmd_vel.angular.z = 0.0;
			}
			else
			if (a[idxMinDist] >= 0) cmd_vel.angular.z = -MAX_ANGULAR_SPEED * 0.5;
			else cmd_vel.angular.z = MAX_ANGULAR_SPEED * 0.5;
		}*/
	}
	}
	free(d);
	free(a);
	free(f);
	if (verbose && !recover) ROS_INFO("(v, w) = (%f, %f) / idxMinDist = %d wl = %f wr = %f", cmd_vel.linear.x, cmd_vel.angular.z, idxMinDist, wl, wr);	
	
	cmd_vel_pub.publish(cmd_vel);
	ros::Duration(0.1).sleep();
}

#define PI2	1.5707963268
#define MINDIST 0.18
#define MAXDIST 6.0
#define NUMLATREAD 8

/*void scanReceived(const sensor_msgs::LaserScan::ConstPtr& pt){
// Versao que usa apenas algumas leituras do LRF emulando um array de sonars
	int size = pt->ranges.size(); // number of points in the scan
	const int nreads = NUMLATREAD+1;
	float angle[nreads];
	const float da = PI2 / (NUMLATREAD / 2);
	float d[nreads];
	int a[nreads];
	bool f[nreads];
	int i, j, idxMinDist = 0, ifront;
	static geometry_msgs::Twist cmd_vel;
	float w1, w2, wl, wr; // left weight (wl) and right weight (wr)
	static int nsinv; // number of consecutive signal inversions in angular velocity
	float w_ = cmd_vel.angular.z; // previous angular velocity
	static double eT;
	static bool recover = false; 

	if (recover && eT <= ros::Time::now().toSec())
	{
		recover = false; // finish recovery
		ROS_INFO("End of recovery rotation");
	}
	
	if (!recover) {
	// compute linear speed
	for (i = 0; i < nreads; i++)
	{
		if (i > 0)
		{
			angle[i] = angle[i-1] + da;
			if (i == NUMLATREAD/2 || i == NUMLATREAD/2+1) angle[i] -= da/2.0;
		}
		else angle[i] = -PI2 + da/2.0;
		j = (angle[i] - pt->angle_min) / pt->angle_increment; // index of LRF reading
		if (pt->ranges[j]<1e-3) d[i] = MAXDIST;
		else if (pt->ranges[j]<=MINDIST) d[i] = 0.0;
		else d[i] = pt->ranges[j];

		if (angle[i] > da/4.0) a[i] = 1;
		else if (angle[i] < -da/4.0) a[i] = -1;
		else { a[i] = 0; ifront = i; }

		if (d[i] < d[idxMinDist]) idxMinDist = i;
		//compute flag (true if in front and nearer than SAFE_DIST_TH)
		if (d[i] < SAFE_DIST_TH) f[i] = true; else f[i] = false; 
		if (verbose) ROS_INFO("%d (%d): %f %f (%.1f) %d %d", i, j, d[i], angle[i], angle[i]/PI2*90, a[i], f[i]);
	}
	if (!f[idxMinDist]) cmd_vel.linear.x = MAX_LINEAR_SPEED;
	else if (d[idxMinDist] > STOP_DIST_TH)
				cmd_vel.linear.x = MAX_LINEAR_SPEED *
							(d[idxMinDist] - STOP_DIST_TH) / (SAFE_DIST_TH - STOP_DIST_TH);
	else cmd_vel.linear.x = 0.0;

	// compute angular speed
	if (!f[idxMinDist]) cmd_vel.angular.z = 0.0;
	else
	{
		wl = 0.0; wr = 0.0;
		for (i = 0; i < nreads; i++)
			if (f[i])
			{
				w1 = fabs(cos(angle[i])); // weight as a function of absolute value of cos(angle): more lateral, less weight
				w2 = (d[i] <= DETOUR_OBST_TH ? 1.25 - d[i] / DETOUR_OBST_TH : 0.0);
				if (a[i] > 0) wl += w1 * w2;
				else if (a[i] < 0) wr += w1 * w2;
			}

		wl*=2.0/NUMLATREAD;
		wr*=2.0/NUMLATREAD;

		if ((wl > DIFF_TH || wr > DIFF_TH) && (wl > wr + DIFF_TH || wl + DIFF_TH < wr))
		{
			nsinv = 0;
			if (wl > wr + DIFF_TH ) // left obstacles have higher weight: rotate clockwise (towards negative angles)
				cmd_vel.angular.z = - MAX_ANGULAR_SPEED * wl;
			else if (wl + DIFF_TH < wr)  // right obstacles have higher weight: rotate anticlockwise (towards positive angles)
				cmd_vel.angular.z = MAX_ANGULAR_SPEED * wr;
		}
		else
		{
			if (verbose) ROS_INFO("Lateral readings have a negligible difference");
			if (d[ifront] >= STOP_DIST_TH)
			{
				w1 = (d[ifront] - STOP_DIST_TH) / (SAFE_DIST_TH - STOP_DIST_TH);
				if (w1 < 0.25 ) w1 = 0.25;
				if (w1 > 1.0) w1 = 1.0;
			 	cmd_vel.linear.x = MAX_LINEAR_SPEED * w1;
				cmd_vel.angular.z = 0.0;
			}
			else {
				if (a[idxMinDist] >= 0) cmd_vel.angular.z = -MAX_ANGULAR_SPEED * 0.5;
				else cmd_vel.angular.z = MAX_ANGULAR_SPEED * 0.5;
				// check if there was a signal inversion in angular velocity
				if (cmd_vel.angular.z * w_ < 0.0) ++nsinv;
				if (nsinv > 1) {
					ROS_WARN(" ****** Deadlock Detected! ******");
					// keep current rotation speed until robot rotates PI
					eT = ros::Time::now().toSec() +
						3.14159/MAX_ANGULAR_SPEED * 2.0;
					recover = true;
					nsinv = 0;
				}
			}
		}
	}
	}

	if (verbose && !recover) ROS_INFO("(v, w) = (%f, %f) / d[ifront] = %f d[idxMinDist] = %f wl = %f wr = %f", cmd_vel.linear.x, cmd_vel.angular.z, d[ifront], d[idxMinDist], wl, wr);	

	cmd_vel_pub.publish(cmd_vel);
	ros::Duration(0.1).sleep(); // desired control frequency
}*/


//Versao que usa todas as leituras do LRF
void scanReceived(const sensor_msgs::LaserScan::ConstPtr& pt){
	int size = pt->ranges.size(); // number of points in the scan
	float *d = (float *) malloc(size * sizeof(float));
	float *angle = (float *) malloc(size * sizeof(float));
	int *a = (int *) malloc(size * sizeof(int));
	bool *f = (bool *) malloc(size * sizeof(bool));
	int i, idxMinDist = 0, ifront;
	geometry_msgs::Twist cmd_vel;
	float w1, w2, wl, wr; // left weight (wl) and right weight (wr)
		
	// remove saturated readings
	for (i = 0; i < size; i++)
		if (pt->ranges[i]<1e-3) d[i] = MAXDIST;
		else if (pt->ranges[i]<MINDIST) d[i] = MINDIST;
		else d[i] = pt->ranges[i];

	// compute linear speed
	for (i = 0; i < size; i++)
	{
		if (i > 0) angle[i] = angle[i-1] + pt->angle_increment;
		else angle[i] = pt->angle_min;
		//compute sensor angular position (+1 left, 0 front, -1 right, -2 behind)
		if (angle[i] > -PI2 && angle[i] < PI2)
		{
			if (angle[i] >= pt->angle_increment) a[i] = 1;
			else if (angle[i] < 0.0) a[i] = -1;
			else { a[i] = 0; ifront = i; }
		} else a[i] = -2;
		//update minimum distance
		if (a[i] > -2 && d[i] < d[idxMinDist]) idxMinDist = i;
		//compute flag (true if in front and nearer than SAFE_DIST_TH)
		if (a[i] > -2 && d[i] < SAFE_DIST_TH) f[i] = true; else f[i] = false; 
		//if (verbose) ROS_INFO("%d : %f %f %d %d", i, d[i], angle[i], a[i], f[i]);
	}
	if (!f[idxMinDist]) cmd_vel.linear.x = MAX_LINEAR_SPEED;
	else if (d[idxMinDist] > STOP_DIST_TH)
				cmd_vel.linear.x = MAX_LINEAR_SPEED *
							(d[idxMinDist] - STOP_DIST_TH) / (SAFE_DIST_TH - STOP_DIST_TH);
	else cmd_vel.linear.x = 0.0;

	// compute angular speed
	if (!f[idxMinDist]) cmd_vel.angular.z = 0.0;
	else
	{
		wl = 0.0; wr = 0.0;
		for (i = 0; i < size; i++)
			if (f[i])
			{
				w1 = fabs(cos(angle[i])); // weight as a function of absolute value of cos(angle): more lateral, less weight
				w2 = (d[i] <= DETOUR_OBST_TH ? 1.25 - d[i] / DETOUR_OBST_TH : 0.0);
				if (a[i] > 0) wl += w1 * w2;
				else if (a[i] < 0) wr += w1 * w2;
			}
		wl*=(2.0 / size);
		wr*=(2.0 / size);

		if ((wl > DIFF_TH || wr > DIFF_TH) && (wl > wr + DIFF_TH || wl + DIFF_TH < wr))
		{
			if (wl > wr + DIFF_TH ) // left obstacles have higher weight: rotate clockwise (towards negative angles)
				cmd_vel.angular.z = - MAX_ANGULAR_SPEED * wl;
			else if (wl + DIFF_TH < wr)  // right obstacles have higher weight: rotate anticlockwise (towards positive angles)
				cmd_vel.angular.z = MAX_ANGULAR_SPEED * wr;
		}
		else
		{
			if (verbose) ROS_INFO("Lateral readings have a negligible difference");
			if (d[ifront] >= STOP_DIST_TH)
			{
				w1 = (d[ifront] - STOP_DIST_TH) / (SAFE_DIST_TH - STOP_DIST_TH);
				if (w1 < 0.25 ) w1 = 0.25;
				if (w1 > 1.0) w1 = 1.0;
			 	cmd_vel.linear.x = MAX_LINEAR_SPEED * w1;
				cmd_vel.angular.z = 0.0;
			}
			else
			if (a[idxMinDist] >= 0) cmd_vel.angular.z = -MAX_ANGULAR_SPEED * 0.5;
			else cmd_vel.angular.z = MAX_ANGULAR_SPEED * 0.5;
		}
	}
	free(angle);
	free(a);
	free(f);
	free(d);
	if (verbose) ROS_INFO("(v, w) = (%f, %f) / d[ifront] = %f d[idxMinDist] = %f wl = %f wr = %f", cmd_vel.linear.x, cmd_vel.angular.z, d[ifront], d[idxMinDist], wl, wr);	
	
	cmd_vel_pub.publish(cmd_vel);
	ros::Duration(0.5).sleep();
}


int main(int argc, char** argv){
  
	ros::init(argc, argv, "random_walk");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	ros::Rate loop_rate(1); //1 Hz
  
	//default parameters
	pn.param("max_linear_speed", MAX_LINEAR_SPEED, 0.20);  	// [m/s]
	pn.param("max_angular_speed", MAX_ANGULAR_SPEED, 1.25);	// [rad/s]
	pn.param("safe_dist_th", SAFE_DIST_TH, 0.75);						// [m]
	pn.param("detour_obst_th", DETOUR_OBST_TH, 0.60);				// [m]
	pn.param("stop_dist_th", STOP_DIST_TH, 0.40);						// [m]	
	pn.param("diff_th", DIFF_TH, 0.02);
	pn.param("verbose", verbose, 1);
	pn.param("useLRF", useLRF, 0);

	//read parameters
	pn.getParam("max_linear_speed", MAX_LINEAR_SPEED);
	pn.getParam("max_angular_speed", MAX_ANGULAR_SPEED);
	pn.getParam("safe_dist_th", SAFE_DIST_TH);
	pn.getParam("detour_obst_th", DETOUR_OBST_TH);
	pn.getParam("stop_dist_th", STOP_DIST_TH);
	pn.getParam("diff_th", DIFF_TH);
	pn.getParam("verbose", verbose);
	pn.getParam("useLRF", useLRF);
	ROS_INFO("Parameters:\n max_linear_speed: %f0.3\n max_angular_speed: %f0.3\n safe_dist_th: %f0.3\n detour_obst_th: %f0.3\n stop_dist_th: %f0.3\n diff_th: %f0.3\n verbose: %d\n useLRF: %d\n ",
	MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED, SAFE_DIST_TH, DETOUR_OBST_TH, STOP_DIST_TH, DIFF_TH, verbose, useLRF); 

	// ROS publishers and subscribers
	ros::Subscriber laser_sub;
	ros::Subscriber sonars_sub;
	if (useLRF)
		laser_sub  = n.subscribe<sensor_msgs::LaserScan>("scan", 1, scanReceived);
 	else sonars_sub  = n.subscribe<sensor_msgs::PointCloud>("sonar", 1, sonarReceived);
	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	if (useLRF) ROS_INFO("Random walk has been started (LRF)");
	else ROS_INFO("Random walk has been started (sonars)");
	ros::spin(); //trigger callbacks and prevents exiting
	return(0);
}

