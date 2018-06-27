
#include <ros/ros.h>
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <opencv2/opencv.hpp>
#include <chrono>

image_transport::Publisher pub_;

using namespace std;
using namespace cv;
using namespace cv::ml;

vector<string> templats;

vector<Vec4i> lane_lines;
int errorDetection = 0;


//Function that receives the images from the camera
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
//Function that detect the signals on the road
cv::Mat detectFigures(cv::Mat dImg);
//We transform the image from sensor_msgs::ImageConstPtr to CV::Mat to use it
cv::Mat transform(const sensor_msgs::ImageConstPtr& msg);
void mergeOverlappingBoxes(std::vector<cv::Rect> &inputBoxes, cv::Mat &image, std::vector<cv::Rect> &outputBoxes);



cv::Mat detectFigures(cv::Mat dImg){

    vector<Rect> rects;

    // Probabilistic Line Transform
    vector<Vec4i> linesP; // will hold the results of the detection

    //We choose the method to search the road signals
    int match_method = 4; //0 - 5

    //We search all possible signals in the main image
    for( size_t i = 0; i < templats.size(); i++ ){

	    cv::Mat result;

	    cv::Mat img_display;

	    //Read the road signal
	    cv::Mat templat = imread(templats[i],CV_LOAD_IMAGE_COLOR);
		
            //If it is not empty
	    if(!templat.empty()){
	    
	    dImg.copyTo( img_display );

	    int result_cols =  dImg.cols - templat.cols + 1;

	    int result_rows = dImg.rows - templat.rows + 1;

	    result.create( result_rows, result_cols, CV_32FC1 );

            //Search the template in the main image
	    matchTemplate( dImg, templat, result, match_method );

	    normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

	    double minVal; double maxVal; Point minLoc; Point maxLoc;

	    Point matchLoc;

	    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

	    //This depends on the method used
	    if( match_method  == 0 || match_method == 1)
	    { matchLoc = minLoc; }
	    else
	    { matchLoc = maxLoc; }

	    //If it is in the floor, approximately in our lane
if ((matchLoc.y>=(dImg.size().height/2)) && (matchLoc.x>=(dImg.size().width/2.5)) && (matchLoc.x<=(dImg.size().width-dImg.size().width/2.5)) ){

		Vec3b colorFigure = dImg.at<Vec3b>(Point((matchLoc.x*2 + templat.cols)/2,(matchLoc.y*2 + templat.rows)/2));

		if(colorFigure.val[0]>=215 && colorFigure.val[1]>=215 && colorFigure.val[2]>=215){
                //We draw a line around the signal detected
	            cv::Rect r(matchLoc, Point( matchLoc.x + templat.cols , matchLoc.y + templat.rows));
		    rects.push_back(r);
		}

	    }

	    }
    }
    vector<Rect> newRects;
    mergeOverlappingBoxes(rects,dImg,newRects);
    for(int i = 0; i<newRects.size(); i++){

	rectangle( dImg, newRects[i], cv::Scalar(0,0,255), 4);

    }

    return dImg;

}

void mergeOverlappingBoxes(std::vector<cv::Rect> &inputBoxes, cv::Mat &image, std::vector<cv::Rect> &outputBoxes)
{
    cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1); // Mask of original image
    cv::Size scaleFactor(10,10); // To expand rectangles, i.e. increase sensitivity to nearby rectangles. Doesn't have to be (10,10)--can be anything
    for (int i = 0; i < inputBoxes.size(); i++)
    {
        cv::Rect box = inputBoxes.at(i) + scaleFactor;
        cv::rectangle(mask, box, cv::Scalar(255), CV_FILLED); // Draw filled bounding boxes on mask
    }

    std::vector<std::vector<cv::Point> > contours;
    // Find contours in mask
    // If bounding boxes overlap, they will be joined by this function call
    cv::findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    cv::Rect max;
    for (int i = 0; i < contours.size(); i++)
    {

	outputBoxes.push_back(cv::boundingRect(contours.at(i)));


    }

    
}


cv::Mat transform(const sensor_msgs::ImageConstPtr& msg){

    cv::Mat dImg =  cv_bridge::toCvShare(msg, "bgr8")->image;

    return dImg;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    auto start = std::chrono::high_resolution_clock::now();

    //We transform the message received to image to be able to work with it
    cv::Mat dImg = transform(msg);

    //We detect the lines and the road signs in the image
    cv::Mat figuresImage = detectFigures(dImg);

    //We transform the results in image to publish it
    sensor_msgs::ImagePtr send = cv_bridge::CvImage(std_msgs::Header(), "bgr8", figuresImage).toImageMsg();

    //We publish the image result
    pub_.publish(send);

    auto finish = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed = finish - start;

    ROS_INFO("Duración: %f",elapsed.count());
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "signs");

  ros::NodeHandle n;

  //We include the symbols we are going to search in the main image
  templats.push_back("/home/jorge/catkin_ws/src/detectSigns/src/Images/1-1.png");
  templats.push_back("/home/jorge/catkin_ws/src/detectSigns/src/Images/2-1.png");
  templats.push_back("/home/jorge/catkin_ws/src/detectSigns/src/Images/3-1.png");
  templats.push_back("/home/jorge/catkin_ws/src/detectSigns/src/Images/4-1.png");
  templats.push_back("/home/jorge/catkin_ws/src/detectSigns/src/Images/5-1.jpg");
  templats.push_back("/home/jorge/catkin_ws/src/detectSigns/src/Images/6-1.jpg");
  templats.push_back("/home/jorge/catkin_ws/src/detectSigns/src/Images/7-1.jpg");

  image_transport::ImageTransport it(n);

  //We get the image from the camera
  image_transport::Subscriber sub = it.subscribe("/kitti_player/color/left/image_raw",1000,imageCallback);

  //We publish the results
  pub_ = it.advertise("/signs",1);  

  ros::spin();

  return 0;
}

