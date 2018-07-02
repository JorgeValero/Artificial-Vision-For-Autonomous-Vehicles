
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
//Funcion que realiza la deteccion de peatones.
cv::Mat detectPeople(cv::Mat dImg);
//Transformamos la imagen del formato sensor_msgs::ImageConstPtr& al formato cv::Mat para procesarla.
cv::Mat transform(const sensor_msgs::ImageConstPtr& msg);
//Funcion que combina areas conjuntas detectadas.
void mergeOverlappingBoxes(std::vector<cv::Rect> &inputBoxes, cv::Mat &image, std::vector<cv::Rect> &outputBoxes);



cv::Mat detectPeople(cv::Mat dImg)
{

    HOGDescriptor hog;

    //Establecemos la deteccion de personas por defecto
    hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());

    //Vector donde almacenaremos los peatones detectados
    vector<Rect> found;

    //Vector donde almacenaremos los peatones discriminados
    vector<Rect> found2;

    //Funcion que realiza la deteccion de peatones
    hog.detectMultiScale(dImg, found, 0, Size(8,8), Size(32,32),1.05,2);

    //Recorremos las areas detectadas
    for(size_t i = 0; i<found.size(); i++)
    {
        //Desechamos las detecciones que se encuentren en el cielo de la imagen
	if(found[i].y>=(dImg.size().height/3)){

	    //Obtenemos el color perteneciente al pixel de en medio de la deteccion
	    Vec3b colorPerson = dImg.at<Vec3b>(Point((found[i].x*2 + found[i].width)/2,(found[i].y*2 + found[i].height)/2));

	    //Desechamos las detecciones cuyo pixel interior sea muy blanco, para evitar la confusion con marcas viales
	    if(colorPerson.val[0]<=190 && colorPerson.val[1]<=190 && colorPerson.val[2]<=190){

	        found2.push_back(found[i]);

	    }
	}
     }

    //Vector donde almacenaremos las areas finales detectadas
    vector<Rect> result;

    //Funcion para combinar las areas solapadas
    mergeOverlappingBoxes(found2,dImg,result);

    //Recorremos las areas finales
    for(int i = 0; i<result.size(); i++)
    {
	//Dibujamos las areas en la imagen
	rectangle(dImg, result[i], cv::Scalar(0,0,0), 3);

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

    //Detectamos los peatones presentes en la imagen y las pintamos
    cv::Mat peopleImage = detectPeople(dImg);

    //Transformamos la imagen con las detecciones a formato sensor_msgs::ImagePtr de ROS para poder enviarla
    sensor_msgs::ImagePtr send = cv_bridge::CvImage(std_msgs::Header(), "bgr8", peopleImage).toImageMsg();

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
  ros::init(argc, argv, "ImageConverter");

  ros::NodeHandle n;

  image_transport::ImageTransport it(n);

  //Creamos un suscriptor que reciba la imagen original de la camara
  image_transport::Subscriber sub = it.subscribe("/kitti_player/color/left/image_raw",1000,imageCallback);

  //Definimos el topico que enviara la imagen con las detecciones
  pub_ = it.advertise("/people",1);  

  ros::spin();

  return 0;
}

