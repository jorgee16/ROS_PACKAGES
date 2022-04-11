#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>


static double angle_from_odometry = 0.0;
const int M = 10;
cv::Mat imgs[M];

void odometryCallback(const nav_msgs::Odometry& msg){
	angle_from_odometry = tf::getYaw(msg.pose.pose.orientation)/M_PI*180.0;
	
    static double angle_from_odometry_at0 = angle_from_odometry;
	
    angle_from_odometry = angle_from_odometry - angle_from_odometry_at0;
	
    if (angle_from_odometry > 180){
		angle_from_odometry -= 360.0;
	}
	if (angle_from_odometry < -180){
		angle_from_odometry += 360.0;
	}
}

void cameraCallback(const sensor_msgs::CompressedImageConstPtr& msg){
    try{
        cv::Mat img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        cv::imshow("Current Image",img);
        cv::waitKey(1);
        cv::cvtColor(img, img, cv::COLOR_RGB2GRAY);
        for(int i =0; i< M-1; i++){
            imgs[i] = imgs[i+1];
        }
        imgs[M-1]= img;

        if (imgs[0].empty()){
            return;
        }

        // static Ptr<ORB> create(int nfeatures=500, float scaleFactor=1.2f, int nlevels=8, int edgeThreshold=31, int firstLevel=0, int WTA_K=2, int scoreType=ORB::HARRIS_SCORE, int patchSize=31, int fastThreshold=20)
        cv::Ptr<cv::ORB> detector = cv::ORB::create(500);

        std::vector<cv::KeyPoint> keypoints[2];
        cv::Mat descriptors[2];
        // void detectAndCompute(InputArray image, InputArray mask, std::vector< KeyPoint > &keypoints, OutputArray descriptors, bool useProvidedKeypoints=false)
        detector->detectAndCompute(imgs[0], cv::noArray(), keypoints[0], descriptors[0]);
        detector->detectAndCompute(imgs[1], cv::noArray(), keypoints[1], descriptors[1]);

        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
        std::vector<std::vector<cv::DMatch>> knn_matches;
        // std::cout << "# descriptors: " << descriptors[0].size() << "  " << descriptors[1].size() << std::endl;
        // void	knnMatch (InputArray queryDescriptors, InputArray trainDescriptors, std::vector< std::vector< DMatch > > &matches, int k, InputArray mask=noArray(), bool compactResult=false) const
        matcher->knnMatch(descriptors[0], descriptors[1], knn_matches, 2);

        std::vector<cv::KeyPoint> nkeypoints[2];
        std::vector<cv::DMatch> gmatches;
        for (int i=0; i<knn_matches.size(); i++){
            if (knn_matches[i][0].distance < 0.7 * knn_matches[i][1].distance){
                gmatches.push_back(knn_matches[i][0]);
                nkeypoints[0].push_back(keypoints[0][knn_matches[i][0].queryIdx]);
                nkeypoints[1].push_back(keypoints[1][knn_matches[i][0].trainIdx]);
            }
        }
        // std::cout << "# good matches: " << gmatches.size() << std::endl;
        cv::Mat img_matches;
        cv::drawMatches(imgs[0], keypoints[0], imgs[0], keypoints[1], gmatches, img_matches);
        cv::imshow("ORB Features", img_matches);
        cv::waitKey(1);

        if (nkeypoints[0].empty() || nkeypoints[1].empty()){
            return;
        }

        cv::Mat camera_mat = (cv::Mat_<double>(3,3) << 381.9017333984375, 0.0, 319.53375244140625, 0.0, 381.3840637207031, 250.10662841796875, 0.0, 0.0, 1.0);

        std::vector<cv::Point2f> v[2];
        cv::KeyPoint::convert(nkeypoints[0], v[0]);
        cv::KeyPoint::convert(nkeypoints[1], v[1]);

        //Mat cv::findEssentialMat(InputArray points1, InputArray points2, InputArray cameraMatrix, int method=RANSAC, double prob=0.999, double threshold=1.0, OutputArray mask=noArray())
        cv::Mat essential_mat = cv::findEssentialMat(cv::InputArray(v[0]),  cv::InputArray(v[1]), camera_mat);
        if (essential_mat.empty() || essential_mat.size() != cv::Size(3,3)){
            std::cerr << "Couldn't calculate essential matrix" << std::endl;
            return;
        }
        // std::cout << essential_mat << std::endl;

        cv::Mat R1, R2, t;
        // void cv::decomposeEssentialMat (InputArray E, OutputArray R1, OutputArray R2, OutputArray t)
        cv::decomposeEssentialMat(essential_mat, R1, R2, t);


        static double angle_from_camera = 0.0;
        double delta_angle1, delta_angle2, delta_angle;

        delta_angle1 = atan2(R1.at<double>(0,2), R1.at<double>(0,0))/M_PI*180;
        delta_angle2 = atan2(R2.at<double>(0,2), R2.at<double>(0,0))/M_PI*180;

        if  (fabs(delta_angle1) < fabs(delta_angle2) ){
            delta_angle = delta_angle1;
        }
        else{
            delta_angle = delta_angle2;
        }

        angle_from_camera += delta_angle;
        printf("%8.2f\t%8.2f\t%8.2f\n", delta_angle, angle_from_camera, angle_from_odometry); 
    }
    catch(cv::Exception& e){
        ROS_ERROR("Error converting image, %s", e.what());
    }

}


int main(int argc, char **argv){
    ros::init(argc,argv,"motion_from_camera");
    ros::NodeHandle nh;

    ros::Subscriber subs_camera = nh.subscribe("/camera/color/image_raw/compressed", 1, cameraCallback);
    ros::Subscriber subs_odom = nh.subscribe("/odometry/filtered", 1, odometryCallback);

    cv::namedWindow("Current Image");

    ros::Rate rate(10);
    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();
    }
}