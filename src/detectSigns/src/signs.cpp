
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


//Funcion que recibe la imagen de la camara.
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
//Funcion que realiza la deteccion de marcas viales.
cv::Mat detectFigures(cv::Mat dImg);
//Transformamos la imagen del formato sensor_msgs::ImageConstPtr& al formato cv::Mat para procesarla.
cv::Mat transform(const sensor_msgs::ImageConstPtr& msg);
//Funcion que combina areas conjuntas detectadas.
void mergeOverlappingBoxes(std::vector<cv::Rect> &inputBoxes, cv::Mat &image, std::vector<cv::Rect> &outputBoxes);



cv::Mat detectFigures(cv::Mat dImg){

    //Vector donde almacenaremos las marcas viales detectadas
    vector<Rect> rects;

    //Eleccion del metodo de matching a elegir
    int match_method = 4; //0 - 5

    //Recorremos las imagenes de referencia
    for( size_t i = 0; i < templats.size(); i++ ){

	    cv::Mat result;

	    cv::Mat img_display;

	    cv::Mat templat = imread(templats[i],CV_LOAD_IMAGE_COLOR);
		
	    if(!templat.empty()){
	    
	    dImg.copyTo( img_display );

	    int result_cols =  dImg.cols - templat.cols + 1;

	    int result_rows = dImg.rows - templat.rows + 1;

	    result.create( result_rows, result_cols, CV_32FC1 );

	    //Realizamos la deteccion de la imagen por referencia en la imagen principal
	    matchTemplate( dImg, templat, result, match_method );

	    //Normalizamos el resultado
	    normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

	    double minVal; double maxVal; Point minLoc; Point maxLoc;

	    Point matchLoc;

	    //Se obtienen las mejores areas en funcion del metodo de matching escogido
	    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

	    if( match_method  == 0 || match_method == 1)
	    { 

		matchLoc = minLoc; 

	    }

	    else
	    { 

		matchLoc = maxLoc; 

	    }

	    //Obtenemos unicamente las detecciones que se encuentren en nuestro carril
            if ((matchLoc.y>=(dImg.size().height/2)) && (matchLoc.x>=(dImg.size().width/2.5)) && (matchLoc.x<=(dImg.size().width-dImg.size().width/2.5)) ){

		//Se obtiene el color del pixel de en medio del area
		Vec3b colorFigure = dImg.at<Vec3b>(Point((matchLoc.x*2 + templat.cols)/2,(matchLoc.y*2 + templat.rows)/2));

		//Se desechan los pixeles que no cumplen las restricciones de color
		if(colorFigure.val[0]>=215 && colorFigure.val[1]>=215 && colorFigure.val[2]>=215){

		    //Se crea el area
	            cv::Rect r(matchLoc, Point( matchLoc.x + templat.cols , matchLoc.y + templat.rows));

		    //Se almacena el area
		    rects.push_back(r);

		}

	    }

	    }
    }

    //Vector donde almacenaremos las areas finales
    vector<Rect> newRects;

    //Se combinan aquellas areas que se solapen
    mergeOverlappingBoxes(rects,dImg,newRects);

    //Recorremos las nuevas areas
    for(int i = 0; i<newRects.size(); i++){

        //Las pintamos en la imagen
	rectangle( dImg, newRects[i], cv::Scalar(0,0,255), 4);

    }

    return dImg;

}

void mergeOverlappingBoxes(std::vector<cv::Rect> &inputBoxes, cv::Mat &image, std::vector<cv::Rect> &outputBoxes)
{
    //Se crea una mascara de la imagen original
    cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);

    //Se define un factor de escala
    cv::Size scaleFactor(10,10);

    //Se recorren las areas detectadas
    for (int i = 0; i < inputBoxes.size(); i++)
    {
        //Se obtienen las areas detectadas en base al factor definido
        cv::Rect box = inputBoxes.at(i) + scaleFactor;

        //Se pintan en la mascara
        cv::rectangle(mask, box, cv::Scalar(255), CV_FILLED);
    }

    //Vector donde almacenaremos las areas finales
    std::vector<std::vector<cv::Point> > contours;

    //Se detectan los contornos presentes en la mascara
    cv::findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    //Se recorren dichos contornos
    for (int i = 0; i < contours.size(); i++)
    {

	//Se almacenan en el vector pasado por referencia
	outputBoxes.push_back(cv::boundingRect(contours.at(i)));


    }

    
}


cv::Mat transform(const sensor_msgs::ImageConstPtr& msg){

    //Transformamos la imagen al formato cv::Mat para poder trabajar con ella
    cv::Mat dImg =  cv_bridge::toCvShare(msg, "bgr8")->image;

    return dImg;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //Medimos el tiempo de ejecucion del procesamiento
    auto start = std::chrono::high_resolution_clock::now();

    //Transformamos la imagen recibida a formato cv::Mat para procesarla
    cv::Mat dImg = transform(msg);

    //Detectamos las marcas viales presentes en la imagen y las pintamos
    cv::Mat figuresImage = detectFigures(dImg);

    //Transformamos la imagen con las detecciones a formato sensor_msgs::ImagePtr de ROS para poder enviarla
    sensor_msgs::ImagePtr send = cv_bridge::CvImage(std_msgs::Header(), "bgr8", figuresImage).toImageMsg();

    //Publicamos dicha imagen en su topico
    pub_.publish(send);

    //Paramos el tiempo de ejecucion
    auto finish = std::chrono::high_resolution_clock::now();

    //Calculamos el tiempo
    std::chrono::duration<double> elapsed = finish - start;

    //Mostramos el resultado
    ROS_INFO("Duración: %f",elapsed.count());
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "signs");

  ros::NodeHandle n;

  //Cargamos las imagenes de referencia, ES NECESARIO CAMBIAR LOS PATHS
  templats.push_back("/home/jorge/catkin_ws/src/detectSigns/src/Images/1-1.png");

  templats.push_back("/home/jorge/catkin_ws/src/detectSigns/src/Images/2-1.png");

  templats.push_back("/home/jorge/catkin_ws/src/detectSigns/src/Images/3-1.png");

  templats.push_back("/home/jorge/catkin_ws/src/detectSigns/src/Images/4-1.png");

  templats.push_back("/home/jorge/catkin_ws/src/detectSigns/src/Images/5-1.jpg");

  templats.push_back("/home/jorge/catkin_ws/src/detectSigns/src/Images/6-1.jpg");

  templats.push_back("/home/jorge/catkin_ws/src/detectSigns/src/Images/7-1.jpg");

  image_transport::ImageTransport it(n);

  //Creamos un suscriptor que reciba la imagen original
  image_transport::Subscriber sub = it.subscribe("/kitti_player/color/left/image_raw",1000,imageCallback);

  //Definimos el topico que enviara la imagen con las detecciones
  pub_ = it.advertise("/signs",1);  

  ros::spin();

  return 0;
}

