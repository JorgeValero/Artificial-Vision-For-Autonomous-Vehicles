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

//Path donde se encuentra el clasificador en cascada, CAMBIAR PATH
String car_cascade_name = "/home/jorge/catkin_ws/src/detectCars/src/cars.xml";

//Clasificador en cascada
CascadeClassifier car_cascade;

//Funcion que recibe la nube de puntos.
void imageCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
//Funcion que detecta las lineas en la imagen.
cv::Mat detectLines(cv::Mat dImg);
//Esta funcion discrimina las lineas para evitar la deteccion de falsos positivos, y divide las lineas en verticales y horizontales.
void discriminateLines(vector<Vec4i> linesP, cv::Mat dImg, vector<Vec4i> *horizontal_lines);
//Esta funcion se encarga de agrupar las lineas pertenecientes a la izquierda y derecha, para obtener solamente dos lineas de carril.
//Para hacerlo se obtiene la media entre ellas.
vector<Vec4i> oneLine(vector<Vec4i> left_lines, vector<Vec4i> right_lines);
//Funcion que se encarga de dibujar las lineas.
cv::Mat drawLines(cv::Mat dImg, vector<Vec4i> linesP, cv::Scalar color);
//Se realiza un algoritmo divide y venceras para comprobar el color de cada pixel perteneciente a la linea.
cv::Vec3b detectColor(cv::Mat dImg, Vec4i l);
//Esta funcion corta la imagen en un triangulo que muestra unicamente nuestro carril, asi logramos una mejor deteccion de las lineas y de las marcas viales.
cv::Mat cropImage(cv::Mat dImg);
//Transformamos la imagen del formato sensor_msgs::Image& al formato cv::Mat para procesarla.
cv::Mat transform(const sensor_msgs::Image& msg);
//Esta funcion se encarga de obtener la imagen procedente de la nube de puntos.
sensor_msgs::Image getImage(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
//Esta funcion se encarga de retornar la posicion 3D de la linea.
void linesPointcloud(cv::Mat dImg, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_msg);
//Esta funcion realiza la deteccion de vehiculos.
cv::Mat detectCars(cv::Mat img);
//Funcion que combina areas conjuntas detectadas.
void mergeOverlappingBoxes(std::vector<cv::Rect> &inputBoxes, cv::Mat &image, std::vector<cv::Rect> &outputBoxes);
//Funcion que realiza la deteccion de marcas viales.
cv::Mat detectFigures(cv::Mat dImg);
//Funcion que realiza la deteccion de peatones.
cv::Mat detectPeople(cv::Mat dImg);
//Se obtienen los puntos pertenecientes a la recta a traves de un algoritmo de divide y venceras
vector<Point> lineVec(cv::Mat dImg, Vec4i l);


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

cv::Mat detectCars(cv::Mat img)
{
  //En el caso de error al cargar el clasificador
  if( !car_cascade.load( car_cascade_name ) ){ printf("--(!)Error loading\n");

          return img; 

  } else {

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

void linesPointcloud(cv::Mat dImg, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_msg)
{
    //Creamos un objeto tipo PointIndices para guardar los indices de los puntos pertenecientes a las lineas detectadas
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    //Se crea un objeto tipo ExtractIndices para extraer puntos de la base de datos
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    //Se crea un vector auxiliar para ir guardando los puntos pertenecientes a las lineas
    std::vector<int> ind;

    //Si han sido detectadas dos lineas
    if(lane_lines.size()==2){

	    //Se obtienen los puntos de inicio y fin de cada linea
	    Vec4i l = lane_lines[0];

	    int idx = l[1] * dImg.cols + l[0];

	    int idx2 = l[3] * dImg.cols + l[2];

	    Vec4i l2 = lane_lines[1];

	    int idx3 = l2[1] * dImg.cols + l2[0];

	    int idx4 = l2[3] * dImg.cols + l2[2];

	    //Se comprueba si los puntos inicio y fin existen en el mapa de puntos
	    if(pcl::isFinite(cloud_msg->points[idx]) && pcl::isFinite(cloud_msg->points[idx2]) && pcl::isFinite(cloud_msg->points[idx3]) && pcl::isFinite(cloud_msg->points[idx4])){ 

		    //Si existen los introducimos
		    inliers->indices.push_back(idx);

		    inliers->indices.push_back(idx2);

		    inliers->indices.push_back(idx3);

		    inliers->indices.push_back(idx4);

	    }

	    //Obtenemos todos los puntos pertenecientes a la linea 1
	    vector<Point> pixeles = lineVec(dImg,l);

	    //Los recorremos
	    for(int i=0;i<pixeles.size();i++){

		//Si existe en la nube de puntos
		if(pcl::isFinite(cloud_msg->points[pixeles[i].y*dImg.cols+pixeles[i].x])){

		    //Se introduce
		    ind.push_back(pixeles[i].y*dImg.cols+pixeles[i].x);

		}

	    }

	    //Obtenemos todos los puntos pertenecientes a la linea 2
	    vector<Point> pixeles2 = lineVec(dImg,l2);

	    //Los recorremos
	    for(int i=0;i<pixeles2.size();i++){

		//Si existe en la nube de puntos
		if(pcl::isFinite(cloud_msg->points[pixeles2[i].y*dImg.cols+pixeles2[i].x])){

		    //Se introduce
		    ind.push_back(pixeles2[i].y*dImg.cols+pixeles2[i].x);

		}

	    }

	    inliers->indices=ind;

	    extract.setInputCloud(cloud_msg);

	    extract.setIndices(inliers);

	    //Se borran todos los puntos excepto los introducidos pertenecientes a las lineas
	    extract.setNegative(false);

	    extract.filter(*cloud_msg);

    //Si se ha detectado solamente una linea
    }else if(lane_lines.size()==1)
    {

	    //Se obtienen sus puntos de inicio y fin
	    Vec4i l = lane_lines[0];

	    int idx = l[1] * dImg.cols + l[0];

	    int idx2 = l[3] * dImg.cols + l[2];

	    //Se comprueba si los puntos inicio y fin de la linea existen en el mapa de puntos
	    if(pcl::isFinite(cloud_msg->points[idx]) && pcl::isFinite(cloud_msg->points[idx2])){ 

		//Si existen los introducimos
	        inliers->indices.push_back(idx);

	        inliers->indices.push_back(idx2);

	    }

	    //Obtenemos todos los puntos pertenecientes a la linea
	    vector<Point> pixeles = lineVec(dImg,l);

	    //Los recorremos
	    for(int i=0;i<pixeles.size();i++){

		//Si existe en la nube de puntos
		if(pcl::isFinite(cloud_msg->points[pixeles[i].y*dImg.cols+pixeles[i].x])){

		    //Se introduce
		    ind.push_back(pixeles[i].y*dImg.cols+pixeles[i].x);

		}

	    }

	    inliers->indices=ind;

	    extract.setInputCloud(cloud_msg);

	    extract.setIndices(inliers);

	    //Se borran todos los puntos excepto los introducidos pertenecientes a las lineas
	    extract.setNegative(false);

	    extract.filter(*cloud_msg);

    }else{
	    //Si no se detectan lineas, se devuelve una nube de puntos vacia
	    extract.setInputCloud(cloud_msg);

	    extract.setIndices(inliers);

	    extract.setNegative(false);

	    extract.filter(*cloud_msg);

    }

}


vector<Point> lineVec(cv::Mat dImg, Vec4i l){

    //Calculamos la distancia entre los pixeles.
    double distancia = sqrt(pow(l[0]-l[2],2)+pow(l[1]-l[3],2));

    //Si la distancia es muy corta, se devuelve el color perteneciente al pixel que se encuentra en el medio.
    if (distancia <= 2){
	vector<Point> vec;
	vec.push_back(Point((int)((l[0]+l[2])/2),(int)((l[1]+l[3])/2)));
	return vec;
    }
    
    //Dividimos la linea en dos partes y se llama a la funcion de manera recursiva
    vector<Point> lineLeft = lineVec(dImg, Vec4i(l[0],l[1],(int)((l[0]+l[2])/2),(int)((l[1]+l[3])/2)));

    vector<Point> lineRight = lineVec(dImg, Vec4i((int)((l[0]+l[2])/2),(int)((l[1]+l[3])/2),l[2],l[3]));

    //Se devuelve el vector final que compone a las dos partes
    lineLeft.insert(lineLeft.end(), lineRight.begin(), lineRight.end());

    return lineLeft;

}


vector<Vec4i> oneLine(vector<Vec4i> left_lines, vector<Vec4i> right_lines){

    vector<Vec4i> lines;

    //Definimos los puntos de las lineas detectadas    
    double MX1 = 0.0, MX2 = 0.0, MY1 = 0.0, MY2 = 0.0;

    double MX3 = 0.0, MX4 = 0.0, MY3 = 0.0, MY4 = 0.0;

    //Si han sido detectadas lineas en el lado izquierdo
    if (left_lines.size()!=0){

        //Recorremos dichas lineas
        for( size_t i = 0; i < left_lines.size(); i++ )
        {
	    //Seleccionamos una
	    Vec4i l = left_lines[i];

    	    //Sumamos sus puntos
	    MX1+=l[0];

	    MY1+=l[1];

	    MX2+=l[2];

	    MY2+=l[3];

        }

	//Una vez hallado el sumatorio de sus puntos, se hace la media
        MX1/=left_lines.size();

        MY1/=left_lines.size();

        MX2/=left_lines.size();

        MY2/=left_lines.size();

	//Se almacena la media de todas las lineas
        lines.push_back(cv::Vec4i(MX1,MY1,MX2,MY2));
    }

    //Si han sido detectadas lineas en el lado derecho
    if (right_lines.size()!=0){

        //Recorremos dichas lineas
        for( size_t i = 0; i < right_lines.size(); i++ )
        {
	    //Seleccionamos una
	    Vec4i l = right_lines[i];

    	    //Sumamos sus puntos
	    MX3+=l[0];

	    MY3+=l[1];

	    MX4+=l[2];

	    MY4+=l[3];
        }

	//Una vez hallado el sumatorio de sus puntos, se hace la media
        MX3/=right_lines.size();

        MY3/=right_lines.size();

        MX4/=right_lines.size();

        MY4/=right_lines.size();

        //Se almacena la media de todas las lineas
        lines.push_back(cv::Vec4i(MX3,MY3,MX4,MY4));
    }

    return lines;

}

cv::Vec3b detectColor(cv::Mat dImg, Vec4i l){

    //Calculamos la distancia entre los pixeles.
    double distancia = sqrt(pow(l[0]-l[2],2)+pow(l[1]-l[3],2));

    //Si la distancia es muy corta, se devuelve el color perteneciente al pixel que se encuentra en el medio.
    if (distancia < 20){
	return dImg.at<Vec3b>(Point((int)((l[0]+l[2])/2),(int)((l[1]+l[3])/2)));
    }
    
    //Dividimos la linea en dos partes y se llama a la funcion de manera recursiva
    Vec3b colorLeft = detectColor(dImg, Vec4i(l[0],l[1],(int)((l[0]+l[2])/2),(int)((l[1]+l[3])/2)));

    Vec3b colorRight = detectColor(dImg, Vec4i((int)((l[0]+l[2])/2),(int)((l[1]+l[3])/2),l[2],l[3]));

    //Se devuelve el color medio
    return Vec3b((int)((colorLeft.val[0]+colorRight.val[0])/2),
	         (int)((colorLeft.val[1]+colorRight.val[1])/2),
                 (int)((colorLeft.val[2]+colorRight.val[2])/2));

}

cv::Mat drawLines(cv::Mat dImg, vector<Vec4i> linesP, cv::Scalar color){

    //Si no hay lineas
    if(linesP.size()==0){

        //Se devuelve la imagen original sin modificar
        return dImg;

    }else{

	//Copiamos la imagen
        cv::Mat cpImg = dImg.clone();

        //Recorremos las lineas detectadas
        for( size_t i = 0; i < linesP.size(); i++ )
        {
	    Vec4i l = linesP[i];

	    //Dibujamos las lineas en la nueva imagen
	    line(cpImg, Point(l[0], l[1]), Point(l[2], l[3]), color, 10);
	    
        }

	//Devolvemos la imagen modificada
        return cpImg;

    }
}

void discriminateLines(vector<Vec4i> linesP, cv::Mat dImg, vector<Vec4i> *horizontal_lines){

    //Vector que almacenara las lineas del carril izquierdo
    vector<Vec4i> left_lines;

    //Vector que almacenara las lineas del carril derecho
    vector<Vec4i> right_lines;

    //Recorremos las lineas detectadas
    for( size_t i = 0; i < linesP.size(); i++ )
    {
	//Seleccionamos la linea
        Vec4i l = linesP[i];

        //Obtenemos el color de la linea
        Vec3b colorLine = detectColor(dImg,l);

        //Si el color cumple con las restricciones de color impuestas
	if(colorLine.val[0]>=165 && colorLine.val[1]>=165 && colorLine.val[2]>=165){

            //Se calcula el angulo de la linea
	    double Angle = atan2(fabsf(l[2] - l[0]), fabsf(l[3] - l[1])) * 180.0 / CV_PI;

            //Si es una linea vertical y se encuentra a la izquierda
            if (Angle<70 && l[0]<dImg.size().width/2 && l[2]<dImg.size().width/2){

		//La almacenamos en el vector de lineas pertenecientes al carril izquierdo
                left_lines.push_back(linesP[i]);

             //Si es una linea vertical y se encuentra a la derecha
             }else if(Angle<70 && l[0]>dImg.size().width/2 && l[2]>dImg.size().width/2){

		//La almacenamos en el vector de lineas pertenecientes al carril derecho
	        right_lines.push_back(linesP[i]);

	     }

            //Si es una linea horizontal y cumple con las restricciones de color impuestas
	    if (Angle>=85 && colorLine.val[0]>=177 && colorLine.val[1]>=177 && colorLine.val[2]>=177){

		//La almacenamos en el vector de lineas horizontales
	        horizontal_lines->push_back(linesP[i]);

	    }	
	}

    }

    //Inicializamos un vector auxiliar de lineas
    vector<Vec4i> aux_lines;
    
    //Agrupamos las lineas pertenecientes a la izquierda y la derecha, obteniendo asi dos lineas de carril, y las almacenamos en dicho vector
    aux_lines = oneLine(left_lines, right_lines);

    //Si al menos se ha detectado una linea y el error no supera un umbral determinado
    if(aux_lines.size()==1 && lane_lines.size()!=0 && errorDetection <=5){

		//Si esta linea es la perteneciente al carril izquierdo
		if(aux_lines[0][0]<(dImg.size().width/2)){

			//Comprobamos si la linea que disponemos es tambien la izquierda
			if(lane_lines[0][0]<(dImg.size().width/2)){

				//Comprobamos si existe la linea del carril derecho
				if(lane_lines.size()==2){

					//Si existe la introducimos
					aux_lines.push_back(lane_lines[1]);
				}
			}else{		

				//Si disponemos de la derecha, introducimos la izquierda
				aux_lines.push_back(lane_lines[0]);

			}
		//Si se trata de la linea perteneciente al carril derecho
		}else{

			//Comprobamos si la que tenemos es del carril izquierdo
			if(lane_lines[0][0]<(dImg.size().width/2)){

			    //Si existe la introducimos
			    aux_lines.push_back(lane_lines[0]);

			}else{

				//Comprobamos si existe la linea del carril izquierdo
				if(lane_lines.size()==2){

					//Si existe la introducimos
					aux_lines.push_back(lane_lines[1]);

				}
			}
		}
    }
    
    //Si hemos detectado dos lineas en esta imagen, el error se resetea
    if (aux_lines.size()==2){

		errorDetection = 0;

    //Si no hemos detectado ninguna linea, se incrementa el error
    }else if(aux_lines.size()!=1){

		errorDetection += 1;

    }

    //Si hemos detectado nuevas lineas o el error es muy alto, se reemplazan las lineas
    if (aux_lines.size()!=0 || errorDetection > 5){

        lane_lines=aux_lines;

    }


}



cv::Mat cropImage(cv::Mat dImg){

    //Se crea una imagen con el mismo tamaño que la imagen original
    cv::Mat mask = cv::Mat::zeros(dImg.rows, dImg.cols, CV_8UC1);

    //Se inicializan una serie de puntos que definiran el area a recortar
    cv::Point corners[1][3];

    //Elegimos el area a recortar
    corners[0][0] = Point(dImg.size().width/5, dImg.size().height);

    corners[0][1] = Point(dImg.size().width/2, dImg.size().height/2);

    corners[0][2] = Point(dImg.size().width-dImg.size().width/5, dImg.size().height);

    const Point* corner_list[1] = { corners[0] };

    int num_points = 3;

    int num_polygons = 1;

    int line_type = 8;

    //Incluimos el area elegida en la imagen creada
    cv::fillPoly(mask, corner_list, &num_points, num_polygons, cv::Scalar(255, 255, 255), line_type);

    //Creamos una imagen con fondo negro que almacenara el resultado
    cv::Mat result(dImg.size(), dImg.type(), cv::Scalar(0,0,0));

    //Copiamos el area seleccionada en la nueva imagen
    dImg.copyTo(result,mask);

    //Devolvemos esta nueva imagen
    return result;

}

cv::Mat detectLines(cv::Mat dImg){

    cv::Mat dst, cdst;

    //Pasamos la imagen original a escala de grises
    cvtColor(dImg, dst, COLOR_RGB2GRAY);

    //Le aplicamos el filtro Canny
    Canny(dst, cdst, 100, 200);

    //Recortamos la imagen para mejorar la deteccion de las lineas
    cv::Mat croppedImage = cropImage(cdst);

    //Vector que almacenara los lineas detectadas
    vector<Vec4i> linesP; 

    //Aplicamos la funcion HoughLine a la imagen
    HoughLinesP(croppedImage, linesP, 6, CV_PI/60, 160, 100, 30);
    
    //Vector que almacenara las lineas horizontales detectadas
    vector<Vec4i> horizontal_lines;
    
    //Funcion que se encarga de discriminar las lineas detectadas
    discriminateLines(linesP,dImg,&horizontal_lines);
 
    //Dibujamos las lineas resultantes en la imagen original, y la devolvemos modificada
    return drawLines(drawLines(dImg, horizontal_lines, cv::Scalar(0,255,0)), lane_lines, cv::Scalar(255,0,0));

}

cv::Mat transform(const sensor_msgs::Image& msg){

    //Transformamos la imagen al formato cv::Mat para poder trabajar con ella
    cv::Mat dImg =  cv_bridge::toCvCopy(msg, "bgr8")->image;

    return dImg;
}

sensor_msgs::Image getImage(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

    sensor_msgs::Image image_;

    try
    {
      //Obtenemos una imagen a partir de la nube de puntos
      pcl::toROSMsg (*cloud_msg, image_); 
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
    //Medimos el tiempo de ejecucion del procesamiento
    auto start = std::chrono::high_resolution_clock::now();
    
    //Obtenemos la imagen procedente de la nube de puntos
    sensor_msgs::Image imgmsg = getImage(cloud_msg);

    //La transformamos a formato cv::Mat para procesarla
    cv::Mat dImg = transform(imgmsg);

    //Detectamos las lineas presentes en la imagen y las pintamos
    cv::Mat linesImage = detectLines(dImg);

    //Detectamos los vehiculos presentes en la imagen y los pintamos
    cv::Mat dImg2 = detectCars(linesImage);

    //Detectamos las marcas viales presentes en la imagen y las pintamos
    cv::Mat dImg3 = detectFigures(dImg2);

    //Detectamos los peatones presentes en la imagen y los pintamos
    cv::Mat dImg4 = detectPeople(dImg3);

    //Creamos una nube de puntos que contendra la posicion 3D de las lineas
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB> ());

    //Obtenemos un puntero de la nube de puntos
    pcl::fromROSMsg (*cloud_msg, *temp); 

    //Imprimimos su posicion 3D en la nube de puntos
    linesPointcloud(dImg, temp);

    //Creamos un mensaje sensor_msgs::PointCloud2 para mandar la nube de puntos a traves de ROS
    sensor_msgs::PointCloud2 resultLines;

    //Transformamos la nube de puntos en el formato de mensajes de ROS para poder enviarla
    pcl::toROSMsg(*temp, resultLines);

    //Transformamos la imagen con las detecciones a formato sensor_msgs::ImagePtr de ROS para poder enviarla
    sensor_msgs::ImagePtr send = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dImg4).toImageMsg();

    //Publicamos dicha imagen en su topico
    pub_.publish(send);

    //Publicamos en su topico la nube de puntos con la posicion en 3D de las lineas
    pub_2.publish(resultLines);

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

  //Cargamos las imagenes de referencia, ES NECESARIO CAMBIAR LOS PATHS
  templats.push_back("/home/jorge/catkin_ws/src/detectSigns/src/Images/1-1.png");

  templats.push_back("/home/jorge/catkin_ws/src/detectSigns/src/Images/2-1.png");

  templats.push_back("/home/jorge/catkin_ws/src/detectSigns/src/Images/3-1.png");

  templats.push_back("/home/jorge/catkin_ws/src/detectSigns/src/Images/4-1.png");

  templats.push_back("/home/jorge/catkin_ws/src/detectSigns/src/Images/5-1.jpg");

  templats.push_back("/home/jorge/catkin_ws/src/detectSigns/src/Images/6-1.jpg");

  templats.push_back("/home/jorge/catkin_ws/src/detectSigns/src/Images/7-1.jpg");

  image_transport::ImageTransport it(n);

  //Creamos un suscriptor que reciba la nube de puntos
  ros::Subscriber sub = n.subscribe("/kitti_player/color/points2",1000,imageCallback);

  //Definimos el topico que enviara la imagen con las detecciones
  pub_ = it.advertise("/detection",1);  

  //Definimos el topico que enviara la nube de puntos con la posicion 3D de las lineas detectadas
  pub_2 = n.advertise<sensor_msgs::PointCloud2> ("/pointcloudLines",1);

  ros::spin();

  return 0;
}

