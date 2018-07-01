
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

String car_cascade_name = "/home/jorge/catkin_ws/src/detectCars/src/cars.xml";

CascadeClassifier car_cascade;

//Function that receives the images from the camera
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
//Function that detect the signals on the road
cv::Mat detectCars(cv::Mat dImg);
//We transform the image from sensor_msgs::ImageConstPtr to CV::Mat to use it
cv::Mat transform(const sensor_msgs::ImageConstPtr& msg);
void mergeOverlappingBoxes(std::vector<cv::Rect> &inputBoxes, cv::Mat &image, std::vector<cv::Rect> &outputBoxes);



cv::Mat detectCars(cv::Mat img)
{
  std::vector<Rect> boxes;
  std::vector<Rect> boxesMedium;
  std::vector<Rect> newBoxes;
  cv::Mat frame_gray;

  cvtColor( img, frame_gray, CV_BGR2GRAY );
  equalizeHist( frame_gray, frame_gray );
  //-- Detect cars
  car_cascade.detectMultiScale( frame_gray, boxes, 1.1, 2/*, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );*/);

  for(size_t i = 0; i<boxes.size(); i++)
  {

      if(boxes[i].height>=80 && boxes[i].width>=80 && boxes[i].height<=150 && boxes[i].width<=150){

          rectangle( img, boxes[i], cv::Scalar(255,0,255), 4);

      }

  }

  return img;
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

    if( !car_cascade.load( car_cascade_name ) ){ printf("--(!)Error loading\n"); 

    //We publish the image result
    pub_.publish(msg);

    }else{

    //We transform the message received to image to be able to work with it
    cv::Mat dImg = transform(msg);

    //We detect the lines and the road signs in the image
    cv::Mat carsImage = detectCars(dImg);

    //We transform the results in image to publish it
    sensor_msgs::ImagePtr send = cv_bridge::CvImage(std_msgs::Header(), "bgr8", carsImage).toImageMsg();

    //We publish the image result
    pub_.publish(send);

    }

    auto finish = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed = finish - start;

    ROS_INFO("Duración: %f",elapsed.count());
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "objects");

  ros::NodeHandle n;

  image_transport::ImageTransport it(n);

  //We get the image from the camera
  image_transport::Subscriber sub = it.subscribe("/kitti_player/color/left/image_raw",1000,imageCallback);

  //We publish the results
  pub_ = it.advertise("/cars",1);  

  ros::spin();

  return 0;
}

