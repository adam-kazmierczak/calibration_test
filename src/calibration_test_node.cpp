#include <ros/ros.h>
#include <sys/types.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include "stdio.h"
#include <fstream>
#include <iostream>
#include<vector>
#include"math.h"
#include "std_msgs/String.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <stdlib.h>


#include "../include/chess.h"

using namespace sensor_msgs;
using namespace message_filters;
using namespace std_msgs;


//----------IMPORTANT!!!!---------------------

std::string srcPath = "./src/calibration_test";

std::string  srcPathlaser= srcPath+"/testing_data/laser.txt";
std::string namePic ="/data/RGB";//name of pictures
std::string nameLas ="/data/laser";//name of laser data
std::string las =srcPath+ nameLas + ".txt";//ścieżka pliku tekstowego
 const char *savelaserPath = las.c_str();


int samplenum= 30;//numbers of samples




//----------ZMIENNE UŻYWANE W FUNCKJACH---------------------------
std::vector<std::vector<double > > laserdata;//ODCZYTANE DANE Z PLIKU TXT
std::string picturesPath = srcPath+"/data";
std::string laserPath = srcPath+"/data/laser.txt";

int a;//how many chess are from left to right
int b;//how many chess are from down to up

    cv::Mat szarak;
    cv::Mat obraz;


 std::string st1 = picturesPath+"/RGB";
 std::string ext = ".png";
 std::string filename;






//-------------------FUNKCJA GŁÓWNA-----------------------------------------------------
void testing(const sensor_msgs::ImageConstPtr& image1)
 {

        static int probka = 1;//numer próbki

        ROS_INFO("Working %d",1);
        cv_bridge::CvImagePtr box;
        box = cv_bridge::toCvCopy(image1, sensor_msgs::image_encodings::BGR8);

        std::stringstream ss;
        ss << probka;
        filename = st1 + ss.str() + ext;
         std::cout << filename << "\n";

      obraz = cv::imread(filename,1 );//TESTOWY OBRAZ

         cv::imshow("view", obraz);//pokaz obrazu
            cv::waitKey(20);


          std::string sample = boost::lexical_cast<std::string>(probka);
         cv::imwrite(srcPath + namePic + sample + ".png", obraz);//zapis obrazu

            int tabliczka = laserdata.at(probka).size();


            std::fstream file;
            file.open(savelaserPath, std::fstream::in | std::fstream::out | std::fstream::app/*std::ios::out|std::ios::ate*/  );
            if( file.is_open() == true)
            {
                    ROS_INFO("\n number of taken picture %d ", probka);

                    for(int i=0; i<tabliczka-1; i++)
                    {
                            file<<laserdata[probka][i]<<", ";
                    }
                    file<<laserdata[probka][tabliczka-1]<<std::endl;

                    file.close();
            }

        if(probka<samplenum)
        {
        probka++;
        }
        else
        {
            //TUTAJ FUNKACJA KALIBRACJI
            //chess();

        cv::waitKey(0);
        calculateTransformation(srcPath);
        //system("./chess");
        ros::shutdown();


        }







 }







//-----------------------------------------------------------------------------------------------
//------------------------------------MAIN-------------------------------------------------------
//------------------------------------------------------------------------------------------------


int main(int argc, char** argv)
{


   std::system("pwd");

  ros::init(argc, argv, "calibration_node");
  std::cout<<"Calibration begin"<<std::endl;
  ros::NodeHandle nh;

ROS_INFO("STARTING PROGRAM %d",1);

//-------READING FILE-----------------------------

    int line_num = 0;
    int  num_ignored = 6;

        std::string line;

        std::ifstream infile(srcPathlaser); // relative directory depends on the build path

        cv::Mat tempM = cv::Mat::zeros(100,100,CV_32FC1);
            cv::imwrite("kicia.jpg", tempM);

        std::cout << argv[0] << std::endl ;


        if(infile.fail()) {
            std::cout << "error loading .txt file for reading" <<std::endl;
            return 0;


        }
        ROS_INFO("STARTING PROGRAM %d",2);
        while (std::getline(infile, line)) {
            std::vector<double> row;
            std::istringstream iss(line);
            std::string val_string;
            int cnt = 0;

            while (iss >> val_string) {
                //std::cout << "Value read " << val_string <<std::endl;
                //std::cout << "cnt vs num_ignored " << cnt << " " << num_ignored << std::endl;
                if(cnt++ >= num_ignored)
                {
                    if(val_string == "inf," || val_string == "nan,")
                    {
                        continue;
                    }
                    double val = ::atof(val_string.c_str());
                    row.push_back(val);
                }else { /*std::cout << "Value was ignored" <<std::endl;*/}
            }
            laserdata.push_back(row);
        }


cv::namedWindow("view");
cv::startWindowThread();


ROS_INFO("CHOSEN TEST OF PROGRAM %d",2);

image_transport::ImageTransport it(nh);
image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, testing);


 ros::spin();
cv::destroyWindow("view");

}
