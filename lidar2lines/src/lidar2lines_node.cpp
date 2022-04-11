#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/plot.hpp>

const double CELL_RESOLUTION = 0.05;

void lidarCallback(const sensor_msgs::LaserScan & scan){
	int img_ssize = (int)(2 * 1.1 * scan.range_max/CELL_RESOLUTION);
	cv::Mat img0;
	img0 = cv::Mat::zeros(img_ssize,img_ssize, CV_8UC1);

    // linhas 
	cv::Mat img_lines;
	img_lines = cv::Mat::zeros(img_ssize, img_ssize, CV_8UC3);
	// circles
	cv::Mat img_circles;
	img_circles = cv::Mat::zeros(img_ssize, img_ssize, CV_8UC3);

	cv::Mat img_corners;
	img_corners = cv::Mat::zeros(img_ssize, img_ssize, CV_8UC3);
	
	double range, bearing;
	for (int i=0; i<scan.ranges.size(); i++){ // Scan Loop
		range = scan.ranges[i];
		bearing = scan.angle_min + scan.angle_increment * i;
		if (std::isfinite(range)){
            int ix = (int) (range*cos(bearing)/CELL_RESOLUTION + img_ssize/2);
			int iy = (int) (range*sin(bearing)/CELL_RESOLUTION + img_ssize/2);
			img0.at<unsigned char>(ix, iy) = 250;
			img_lines.at <cv::Vec3b>(ix, iy).val[1] = 250 ;
			// into scan Loop
			img_circles.at<cv::Vec3b>(ix, iy).val[1] = 250;	
			img_corners.at<cv::Vec3b>(ix, iy).val[1] = 250;
		}
	} // End Scan Loop
	cv::Mat img1;
	cv::resize(img0, img1, img0.size()*3, cv::INTER_LINEAR);
	cv::imshow("Lidar Scan", img1);
	cv::waitKey(1);

	cv::Mat img2 = img0;
	std::vector<cv::Vec4i> lines;
	// cv::HoughLinesP (InputArray image, OutputArray lines, double rho, double theta, int threshold, double minLineLength=0, double maxLineGap=0)
	cv::HoughLinesP(img2, lines, 1, 4.0/180.0*M_PI, 5, 5, 25);

	std::cerr << "# line (P) segments: " << lines.size() << std::endl;
	for (int i=0; i < lines.size(); i++){
		cv::line(img_lines, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(255, 0, 0), 1, 8);
		double theta = atan2(lines[i][3]-lines[i][1], lines[i][2]-lines[i][0]) + M_PI/2.0;
		double rho = abs( (lines[i][2] - lines[i][0])*lines[i][1] - lines[i][0]*(lines[i][3] - lines[i][1]))/ sqrt((lines[i][3]-lines[i][1])*(lines[i][3]-lines[i][1]) + (lines[i][2]-lines[i][0])*(lines[i][2]-lines[i][0]));
		std::cout << "- \t" << rho << "\t" << theta/M_PI*180 << std::endl;
	}
	cv::resize(img_lines, img_lines, img_lines.size()*3, cv::INTER_LINEAR);
	cv::imshow("Detected Lines", img_lines);
	cv::waitKey(1);

	cv::Mat img4 = img0;
	std::vector<cv::Vec4f> circles; // (x,y, radius, votes)
	// cv::HoughCircles (InputArray image, OutputArray circles, int method, double dp, double minDist, double param1=100, double param2=100, int minRadius=0, int maxRadius=0)
	cv::HoughCircles(img4, circles, cv::HOUGH_GRADIENT, 1, 5, 100, 12, 2, 20);
	std::cout << "Num circles: " << circles.size() << std::endl;

	// Detect Circles using Hough
	for(int i=0; i<circles.size(); i++){
		cv::Point center(circles[i][0], circles[i][1]);
		int radius = circles[i][2];
		circle(img_circles, center, radius, cv::Scalar(255,0,0), 1);
	}
	cv::resize(img_circles, img_circles, img_circles.size()*3, cv::INTER_LINEAR);
	cv::imshow("Detected Circles", img_circles);
	cv::waitKey(1);

	cv::Mat datax(1, scan.ranges.size(), CV_64F);
	cv::Mat datay(1, scan.ranges.size(), CV_64F);

	for(int i=0; i<scan.ranges.size(); i++){
		if (std::isfinite(scan.ranges[i])){
			datax.at<double>(i) = scan.angle_min + scan.angle_increment * i;
			datay.at<double>(i) = scan.ranges[i];
		}
		else{
			datax.at<double>(i) = scan.angle_min + scan.angle_increment * i;
			datay.at<double>(i) = std::numeric_limits<double>::quiet_NaN();
		}
	}
	cv::Mat plot_scan;
	cv::Ptr<cv::plot::Plot2d> plot = cv::plot::Plot2d::create(datax, datay);
	plot->render(plot_scan);
	cv::imshow("Lidar scan", plot_scan);
	cv::waitKey(1); 

	cv::Mat img5 = img0;
	cv::Mat corners, dilated_corners;
	cv::preCornerDetect(img5, corners, 3);
	//cv::dilate(corners, dilated_corners, cv::Mat());
	cv::dilate(corners, dilated_corners, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)));
	cv::Mat corner_mask = (corners == dilated_corners);

	for (int i=0; i<corner_mask.size().height; i++){
		for (int j=0; j<corner_mask.size().width; j++){
			img_corners.at<cv::Vec3b>(i, j).val[2] = (corner_mask.at<bool>(i,j) ? 0 : 255);	
		}
	}
	cv::resize(img_corners, img_corners, img_corners.size()*3, cv::INTER_LINEAR);
	cv::imshow("Detected Corners", img_corners);
	cv::waitKey(1);

}

int main(int argc, char **argv){
	ros::init(argc, argv, "lidar2lines");
	ros::NodeHandle nh;

	ros::Subscriber subs_lidar = nh.subscribe("/scan", 10, lidarCallback);

	cv::namedWindow("Lidar Scan");
	cv::namedWindow("Detected Lines");
	cv::namedWindow("Detected Circles");
	cv::namedWindow("Plot Retardado");
	cv::namedWindow("Detected Corners");

	ros::Rate rate(10);
	while(ros::ok()){
		rate.sleep();
		ros::spinOnce();
	}
}
