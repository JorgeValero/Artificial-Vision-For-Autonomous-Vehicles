
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

//Funcion que recibe la imagen de la camara.
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
//Esta funcion realiza la deteccion de vehiculos.
cv::Mat detectCars(cv::Mat dImg);
//Transformamos la imagen del formato sensor_msgs::ImageConstPtr& al formato cv::Mat para procesarla.
cv::Mat transform(const sensor_msgs::ImageConstPtr& msg);
//Funcion que combina areas conjuntas detectadas.
void mergeOverlappingBoxes(std::vector<cv::Rect> &inputBoxes, cv::Mat &image, std::vector<cv::Rect> &outputBoxes);



cv::Mat detectCars(cv::Mat img)
{
  //Vector que almacenara las areas o vehiculos iniciales detectados
  std::vector<Rect> boxes;

  //Vector que almacenara las areas o vehiculos finales discriminadas
  std::vector<Rect> newBoxes;

  //Imagen donde se guardara la imagen original pasada a escala de grises
  cv::Mat frame_gray;

  //Transformar la imagen original en color a escala de grises
  cvtColor( img, frame_gray, CV_BGR2GRAY );

  //Obtener sus histogramas de intensidad
  equalizeHist( frame_gray, frame_gray );

  //Aplicacion del clasificador en cascada a la imagen en escala de grises
  car_cascade.detectMultiScale( frame_gray, boxes, 1.1, 2);

  //Combinacion de las areas detectadas que se solapen
  mergeOverlappingBoxes(boxes,img,newBoxes);

  //Recorremos dichas areas
  for(size_t i = 0; i<newBoxes.size(); i++)
  {

      //Aquellas areas cuyo tamaño cumplan las restricciones se pintan en la imagen
      if(newBoxes[i].height>=80 && newBoxes[i].width>=80 && newBoxes[i].height<=150 && newBoxes[i].width<=150){

	  //Se pinta el area en la imagen
          rectangle( img, newBoxes[i], cv::Scalar(255,0,255), 4);

      }

  }

  return img;

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

    //En el caso de error al cargar el clasificador
    if( !car_cascade.load( car_cascade_name ) ){ printf("--(!)Error loading\n");

    //Enviamos la imagen original sin realizar ningun tipo de procesamiento
    pub_.publish(msg);

    }else{

    //Transformamos la imagen recibida a formato cv::Mat para procesarla
    cv::Mat dImg = transform(msg);

    //Detectamos los vehiculos presentes en la imagen y los pintamos
    cv::Mat carsImage = detectCars(dImg);

    //Transformamos la imagen con las detecciones a formato sensor_msgs::ImagePtr de ROS para poder enviarla
    sensor_msgs::ImagePtr send = cv_bridge::CvImage(std_msgs::Header(), "bgr8", carsImage).toImageMsg();

    //Publicamos dicha imagen en su topico
    pub_.publish(send);

    }

    //Paramos el tiempo de ejecucion
    auto finish = std::chrono::high_resolution_clock::now();

    //Calculamos el tiempo
    std::chrono::duration<double> elapsed = finish - start;

    //Mostramos el resultado
    ROS_INFO("Duración: %f",elapsed.count());
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "objects");

  ros::NodeHandle n;

  image_transport::ImageTransport it(n);

  //Creamos un suscriptor que reciba la imagen original de la camara
  image_transport::Subscriber sub = it.subscribe("/kitti_player/color/left/image_raw",1000,imageCallback);

  //Definimos el topico que enviara la imagen con las detecciones
  pub_ = it.advertise("/cars",1);  

  ros::spin();

  return 0;
}

