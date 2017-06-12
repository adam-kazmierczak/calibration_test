//
// Created by jakub on 23.05.17.
//

#include "../include/chess.h"

#include <iostream>

#include <opencv2/opencv.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../tinyXML/tinyxml2.h"
#include <Eigen/Dense>

//#include "getobj.h"

#include "../src/camera_calibration.cpp"
#include "../include/Calibration.h"
#include <vector>
#include <algorithm>

using namespace cv;
using namespace std;
using Eigen::MatrixXd;

RNG rng(12345);

void calculateTransformation(std::string dir)
{
//    int res = opencvCameraCalibration();

    Calibration c(dir + "/xml/CamCalibConfig.xml");
    c.readLaserSetsTXT(dir + "/data/laser.txt");
    c.readValidIDs(dir + "/xml/validFramesIDs.xml");
    c.pickValidLaserSets();
    c.loadFramesData(dir + "/xml/out_camera_data.xml", dir + "/xml/CamCalibConfig.xml");
    c.convValidLaserSets();
    c.wrapDataToItems();
    c.printValidLaserXY();
    c.eq.finCalc();
}
//    tinyxml2::XMLDocument config;
//    config.LoadFile("../xml/calibrationConfig.xml");
//    if (config.ErrorID()) {
//        std::cout << "unable to load config file.\n";
//    }
//    int frames_num = config.FirstChildElement("Config")->FirstChildElement("frames")->IntAttribute("number");
//    cout << "This is frames_num: " << frames_num << endl;

