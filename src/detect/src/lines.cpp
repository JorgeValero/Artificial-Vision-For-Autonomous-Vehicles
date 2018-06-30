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
#include <vector>
#include <chrono>

image_transport::Publisher pub_;
ros::Publisher pub_2;

using namespace cv;
using namespace std;

vector<string> templats;

vector<Vec4i> lane_lines;
int errorDetection = 0;


//Function that receives the images from the camera
void imageCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
//Function that detect the lines on the image
cv::Mat detectLines(cv::Mat dImg);
//This function discriminates the lines to skip bugs, and divide the lines in vertical or horizontal lines
void discriminateLines(vector<Vec4i> linesP, cv::Mat dImg, vector<Vec4i> *horizontal_lines);
//With all the lines detected, this function do one line at the left and one line at the right.
//To do that, we calcule the middle line at the left, and the middle line at the right.
vector<Vec4i> oneLine(vector<Vec4i> left_lines, vector<Vec4i> right_lines);
//Function that draw the lines
cv::Mat drawLines(cv::Mat dImg, vector<Vec4i> linesP, cv::Scalar color);
//We divide the line in two parts until the distance is very short, and we see if the line is more or less white
cv::Vec3b detectColor(cv::Mat dImg, Vec4i l);
//We crop the image in a triangle to detect only my lane, for helping the detection of lines and signals later
cv::Mat cropImage(cv::Mat dImg);
//We transform the image from sensor_msgs::ImageConstPtr to CV::Mat to use it
cv::Mat transform(const sensor_msgs::Image& msg);
sensor_msgs::Image getImage(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
void linesPointcloud(cv::Mat dImg, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_msg);


void linesPointcloud(cv::Mat dImg, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_msg)
{
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    std::vector<int> ind;

    if(lane_lines.size()==2){

	    Vec4i l = lane_lines[0];
	    int idx = l[1] * dImg.cols + l[0];
	    int idx2 = l[3] * dImg.cols + l[2];

	    Vec4i l2 = lane_lines[1];
	    int idx3 = l2[1] * dImg.cols + l2[0];
	    int idx4 = l2[3] * dImg.cols + l2[2];

	    if(pcl::isFinite(cloud_msg->points[idx]) && pcl::isFinite(cloud_msg->points[idx2]) && pcl::isFinite(cloud_msg->points[idx3]) && pcl::isFinite(cloud_msg->points[idx4])){ 

		    float den1 = l[2] - l[0];

		    float den2 = l[3] - l[1];

		    float den3 = l2[2] - l2[0];

		    float den4 = l2[3] - l2[1];

		    for(int i=0; i<dImg.rows; i++){

		        for(int j=0; j<dImg.cols; j++){

			    int p1=((i-l[0])/(den1));
			    int p2=((j-l[1])/(den2));
			    int p3=((i-l2[0])/(den3));
			    int p4=((j-l2[1])/(den4));
		
			    if(p1==p2 && pcl::isFinite(cloud_msg->points[p2 * dImg.cols + p1])){
				
				ind.push_back(p2 * dImg.cols + p1);

		            }

			    if(p3==p4 && pcl::isFinite(cloud_msg->points[p4 * dImg.cols + p3])){
				
				ind.push_back(p4 * dImg.cols + p3);

		            }
		        }

		    }

		    inliers->indices=ind;
		    inliers->indices.push_back(idx);
		    inliers->indices.push_back(idx2);
		    inliers->indices.push_back(idx3);
		    inliers->indices.push_back(idx4);


		    extract.setInputCloud(cloud_msg);
		    extract.setIndices(inliers);
		    extract.setNegative(false);
		    extract.filter(*cloud_msg);

	    }else{

		    extract.setInputCloud(cloud_msg);
		    extract.setIndices(inliers);
		    extract.setNegative(false);
		    extract.filter(*cloud_msg);

	    }

    }else if(lane_lines.size()==1)
    {

	    Vec4i l = lane_lines[0];

	    int idx = l[1] * dImg.cols + l[0];

	    int idx2 = l[3] * dImg.cols + l[2];


	    if(pcl::isFinite(cloud_msg->points[idx]) && pcl::isFinite(cloud_msg->points[idx2])){ 

		    float den1 = l[2] - l[0];

		    float den2 = l[3] - l[1];

		    for(int i=0; i<dImg.rows; i++){

		        for(int j=0; j<dImg.cols; j++){

   			    int p1=((i-l[0])/(den1));

			    int p2=((j-l[1])/(den2));

			    if(p1==p2 && pcl::isFinite(cloud_msg->points[p2 * dImg.cols + p1])){
			
				ind.push_back(p2 * dImg.cols + p1);

		            }
		        }

		    }


		    inliers->indices=ind;
		    inliers->indices.push_back(idx);
		    inliers->indices.push_back(idx2);



		    extract.setInputCloud(cloud_msg);
		    extract.setIndices(inliers);
		    extract.setNegative(false);
		    extract.filter(*cloud_msg);

	    }else{

		    extract.setInputCloud(cloud_msg);
		    extract.setIndices(inliers);
		    extract.setNegative(false);
		    extract.filter(*cloud_msg);

	    }

    }else{

	    extract.setInputCloud(cloud_msg);
	    extract.setIndices(inliers);
	    extract.setNegative(false);
	    extract.filter(*cloud_msg);

    }

}


vector<Vec4i> oneLine(vector<Vec4i> left_lines, vector<Vec4i> right_lines){

    vector<Vec4i> lines;
    
    //We initialize the points of the initial and final point of the right line
    //and the points of the initial and final point of the left line
    double MX1 = 0.0, MX2 = 0.0, MY1 = 0.0, MY2 = 0.0;
    double MX3 = 0.0, MX4 = 0.0, MY3 = 0.0, MY4 = 0.0;

    //If it is the left line
    if (left_lines.size()!=0){
        //We calcule the mean line
        for( size_t i = 0; i < left_lines.size(); i++ )
        {
	    Vec4i l = left_lines[i];
	    MX1+=l[0];
	    MY1+=l[1];
	    MX2+=l[2];
	    MY2+=l[3];

        }

        MX1/=left_lines.size();
        MY1/=left_lines.size();
        MX2/=left_lines.size();
        MY2/=left_lines.size();
        lines.push_back(cv::Vec4i(MX1,MY1,MX2,MY2));
    }
    //If it is the right line
    if (right_lines.size()!=0){
        //We calcule the mean line
        for( size_t i = 0; i < right_lines.size(); i++ )
        {
	    Vec4i l = right_lines[i];
	    MX3+=l[0];
	    MY3+=l[1];
	    MX4+=l[2];
	    MY4+=l[3];
        }

        MX3/=right_lines.size();
        MY3/=right_lines.size();
        MX4/=right_lines.size();
        MY4/=right_lines.size();
        lines.push_back(cv::Vec4i(MX3,MY3,MX4,MY4));
    }

    return lines;

}

cv::Vec3b detectColor(cv::Mat dImg, Vec4i l){

    //We calcule the distance of the line
    double distancia = sqrt(pow(l[0]-l[2],2)+pow(l[1]-l[3],2));

    //If the distance is very short, we return the color of the medium point of the line
    if (distancia < 20){
	return dImg.at<Vec3b>(Point((int)((l[0]+l[2])/2),(int)((l[1]+l[3])/2)));
    }
    
    //We divide the line in two parts
    Vec3b colorLeft = detectColor(dImg, Vec4i(l[0],l[1],(int)((l[0]+l[2])/2),(int)((l[1]+l[3])/2)));

    Vec3b colorRight = detectColor(dImg, Vec4i((int)((l[0]+l[2])/2),(int)((l[1]+l[3])/2),l[2],l[3]));

    //We return the mean color
    return Vec3b((int)((colorLeft.val[0]+colorRight.val[0])/2),
	         (int)((colorLeft.val[1]+colorRight.val[1])/2),
                 (int)((colorLeft.val[2]+colorRight.val[2])/2));

}

cv::Mat drawLines(cv::Mat dImg, vector<Vec4i> linesP, cv::Scalar color){

    //If there are not lines
    if(linesP.size()==0){

        //We return the original image
        return dImg;

    }else{

        cv::Mat cpImg = dImg.clone();

        //We check the lines detected
        for( size_t i = 0; i < linesP.size(); i++ )
        {
	    Vec4i l = linesP[i];

	    //Draw the lines
	    line(cpImg, Point(l[0], l[1]), Point(l[2], l[3]), color, 10);
	    
        }

        return cpImg;

    }
}

void discriminateLines(vector<Vec4i> linesP, cv::Mat dImg, vector<Vec4i> *horizontal_lines){

    vector<Vec4i> left_lines;

    vector<Vec4i> right_lines;

    //We check every line detected
    for( size_t i = 0; i < linesP.size(); i++ )
    {
        Vec4i l = linesP[i];

        //We detect the color of the line
        Vec3b colorLine = detectColor(dImg,l);

        //If it is more or less white
	if(colorLine.val[0]>=165 && colorLine.val[1]>=165 && colorLine.val[2]>=165){

            //We calcule the angle of the line
	    double Angle = atan2(fabsf(l[2] - l[0]), fabsf(l[3] - l[1])) * 180.0 / CV_PI;

            //If it is vertical line and it is in the left
            if (Angle<70 && l[0]<dImg.size().width/2 && l[2]<dImg.size().width/2){

                left_lines.push_back(linesP[i]);

             //If it is vertical line and it is in the right
             }else if(Angle<70 && l[0]>dImg.size().width/2 && l[2]>dImg.size().width/2){

	        right_lines.push_back(linesP[i]);

	     }

            //If it is horizontal line and it is white
	    if (Angle>=87 && colorLine.val[0]>=180 && colorLine.val[1]>=180 && colorLine.val[2]>=180){

	        horizontal_lines->push_back(linesP[i]);

	    }	
	}

    }

    //We initialize a new vector of lines
    vector<Vec4i> aux_lines;
    
    //We store the result lines (left and right)
    aux_lines = oneLine(left_lines, right_lines);

    //If at least there is one line and the error is not too much high
    if(aux_lines.size()==1 && lane_lines.size()!=0 && errorDetection <=5){
		//If the aux line detected is the left line
		if(aux_lines[0][0]<(dImg.size().width/2)){
			//We check if the line of lane line is left line too
			if(lane_lines[0][0]<(dImg.size().width/2)){
				//We check if the right line exists
				if(lane_lines.size()==2){
					//We introduce the right line
					aux_lines.push_back(lane_lines[1]);
				}
			}else{		
				//If it is the right line, we include it
				aux_lines.push_back(lane_lines[0]);
			}
		//If it is the right line
		}else{
			//If it is the left line
			if(lane_lines[0][0]<(dImg.size().width/2)){
			//We include it
			aux_lines.push_back(lane_lines[0]);
			}else{
				//if it is the right line, we check if the left line exists
				if(lane_lines.size()==2){
					//and include it
					aux_lines.push_back(lane_lines[1]);
				}
			}
		}
    }
    
    //If we detected the two lines, the error disappear
    if (aux_lines.size()==2){

		errorDetection = 0;

    //If we didn't detect any lines, we increment the error
    }else if(aux_lines.size()!=1){

		errorDetection += 1;

    }
    //If the error is very high we have to change the lines, or if the aux lines is not empty
    if (aux_lines.size()!=0 || errorDetection > 5){

        lane_lines=aux_lines;

    }


}



cv::Mat cropImage(cv::Mat dImg){

    //Create a black image with the same size of dImg
    cv::Mat mask = cv::Mat::zeros(dImg.rows, dImg.cols, CV_8UC1);

    cv::Point corners[1][3];

    //We choose the area we want to show
    corners[0][0] = Point(dImg.size().width/5, dImg.size().height);

    corners[0][1] = Point(dImg.size().width/2, dImg.size().height/2);

    corners[0][2] = Point(dImg.size().width-dImg.size().width/5, dImg.size().height);

    const Point* corner_list[1] = { corners[0] };

    int num_points = 3;

    int num_polygons = 1;

    int line_type = 8;

    //We include the area we chose previously
    cv::fillPoly(mask, corner_list, &num_points, num_polygons, cv::Scalar(255, 255, 255), line_type);

    //We create a image result
    cv::Mat result(dImg.size(), dImg.type(), cv::Scalar(0,0,0));

    //Copy the result in the new image
    dImg.copyTo(result,mask);

    return result;

}

cv::Mat detectLines(cv::Mat dImg){

    cv::Mat dst, cdst;

    //We change the color of the image to black and white
    cvtColor(dImg, dst, COLOR_RGB2GRAY);

    Canny(dst, cdst, 100, 200);

    //We crop the image to detect the lines and road signals better
    cv::Mat croppedImage = cropImage(cdst);

    // Probabilistic Line Transform
    vector<Vec4i> linesP; // will hold the results of the detection

    //Function to detect lines
    HoughLinesP(croppedImage, linesP, 6, CV_PI/60, 160, 100, 30); // runs the actual detection
    
    vector<Vec4i> horizontal_lines;
    
    //We discriminate the lines to remove bugs
    discriminateLines(linesP,dImg,&horizontal_lines);
 
    //We draw the lines in the image result
    return drawLines(drawLines(dImg, horizontal_lines, cv::Scalar(0,255,0)), lane_lines, cv::Scalar(255,0,0));

}

cv::Mat transform(const sensor_msgs::Image& msg){

    cv::Mat dImg =  cv_bridge::toCvCopy(msg, "bgr8")->image;

    return dImg;
}

sensor_msgs::Image getImage(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

    sensor_msgs::Image image_;

    try
    {
      pcl::toROSMsg (*cloud_msg, image_); //convert the cloud
    }
    catch (std::runtime_error e)
    {
      ROS_ERROR_STREAM("Error in converting cloud to image message: "
                        << e.what());
    }

    return image_;

}

void imageCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

    auto start = std::chrono::high_resolution_clock::now();

    sensor_msgs::Image imgmsg = getImage(cloud_msg);

    //We transform the message received to image to be able to work with it
    cv::Mat dImg = transform(imgmsg);

    //We detect the lines
    cv::Mat linesImage = detectLines(dImg);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB> ());

    pcl::fromROSMsg (*cloud_msg, *temp); 

    linesPointcloud(dImg, temp);

    sensor_msgs::PointCloud2 resultLines;

    pcl::toROSMsg(*temp, resultLines);

    //We transform the results in image to publish it
    sensor_msgs::ImagePtr send = cv_bridge::CvImage(std_msgs::Header(), "bgr8", linesImage).toImageMsg();

    //We publish the image result
    pub_.publish(send);

    pub_2.publish(resultLines);

    auto finish = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed = finish - start;

    ROS_INFO("Duración: %f",elapsed.count());
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lines");

  ros::NodeHandle n;

  image_transport::ImageTransport it(n);

  //We get the image from the camera
  ros::Subscriber sub = n.subscribe("/kitti_player/color/points2",1000,imageCallback);

  //We publish the results
  pub_ = it.advertise("/lines",1);  

  pub_2 = n.advertise<sensor_msgs::PointCloud2> ("/pointcloudLines",1);

  ros::spin();

  return 0;
}

