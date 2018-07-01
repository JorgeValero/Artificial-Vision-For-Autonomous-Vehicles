
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
//Function that detect the lines on the image
cv::Mat detectPeople(cv::Mat dImg);
cv::Mat transform(const sensor_msgs::ImageConstPtr& msg);
void mergeOverlappingBoxes(std::vector<cv::Rect> &inputBoxes, cv::Mat &image, std::vector<cv::Rect> &outputBoxes);



cv::Mat detectPeople(cv::Mat dImg)
{

    /// Set up the pedestrian detector --> let us take the default one
    HOGDescriptor hog;
    hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());

    /// Set up tracking vector
    vector<Point> track;

    vector<Rect> found;

    vector<Rect> found2;

    hog.detectMultiScale(dImg, found, 0, Size(8,8), Size(32,32),1.05,2);

    /// draw detections and store location
    for( size_t i = 0; i < found.size(); i++ )
    {
	Rect r = found[i];

	for(size_t j = 0; j<found.size(); j++)
	{
	    if(j!=i && (r & found[j])==r){
		break;}

	    if(found[i].y>=(dImg.size().height/3)){

		    Vec3b colorPerson = dImg.at<Vec3b>(Point((found[i].x*2 + found[i].width)/2,(found[i].y*2 + found[i].height)/2));

		    if(colorPerson.val[0]<=180 && colorPerson.val[1]<=180 && colorPerson.val[2]<=180){

	                found2.push_back(found[i]);

	     	    }
	     }

	}

     }

    vector<Rect> result;

    mergeOverlappingBoxes(found2,dImg,result);

    for(int i = 0; i<result.size(); i++)
    {

	rectangle(dImg, result[i], cv::Scalar(0,0,0), 3);

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
    cv::Mat peopleImage = detectPeople(dImg);

    //We transform the results in image to publish it
    sensor_msgs::ImagePtr send = cv_bridge::CvImage(std_msgs::Header(), "bgr8", peopleImage).toImageMsg();

    //We publish the image result
    pub_.publish(send);

    auto finish = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed = finish - start;

    ROS_INFO("Duración: %f",elapsed.count());
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ImageConverter");

  ros::NodeHandle n;

  image_transport::ImageTransport it(n);

  //We get the image from the camera
  image_transport::Subscriber sub = it.subscribe("/kitti_player/color/left/image_raw",1000,imageCallback);

  //We publish the results
  pub_ = it.advertise("/people",1);  

  ros::spin();

  return 0;
}

