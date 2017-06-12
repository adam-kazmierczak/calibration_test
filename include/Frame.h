//
// Created by jakub on 24.05.17.
//

#ifndef TESTCV_FRAME_H
#define TESTCV_FRAME_H

#include <iostream>

#include <deque>

#include <opencv2/opencv.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include "../tinyXML/tinyxml2.h"
#include "getobj.h"
#include "../include/Calibration.h"
#include "../include/Equation.h"

class LaserDevice;
class  Frame {
    //variables that are the same for all instances of a class
private:
//    cv::Mat rgb;
//    cv::Mat undist_rgb;
//B: changed to Point3f
    std::vector<Point3f> laserValidPtsXY; // all laserpoints in cartesian space
    std::vector<float> laserValidPtsThetas; // all laserpoints in cartesian space
    std::vector<double> laserPtsRfi; // all laserpoints in cartesian space
    std::deque<Point3f> boardLaserPoints;
    std::vector<Point2f> laserChess; // laserpoints selected manually that belong to a chessboard;
    std::vector<double> rvec; // Rodriguez rotation vector
    cv::Mat rotMatcv;
    Eigen::Matrix<double,3,3> rotMatE;
    std::vector<double> tvec; // translation vector
    Eigen::Vector3d tvecE;
    float angleXoZ;
    cv::Vec6f lineProps;
    Eigen::Vector3d N;
public:
    const static bool verbose = true;
    Frame(const vector<double> &_laserPtsRfi);
    Frame(const vector<Point3f> &laserPts, const vector<double> &rvec, const vector<double> &tvec);
    void calcLaserXY(LaserDevice &l);
    void findLaserBoard();
    void calcAngle();

    void setRvec(const vector<double> &rvec);

    void setTvec(const vector<double> &tvec);

    float getAngleXoZ() const;

    void draw(cv::Mat &screen);

    void calcEQandN(Equation &eq);

    const vector<double> &getTvec() const;
};

#endif //TESTCV_FRAME_H
