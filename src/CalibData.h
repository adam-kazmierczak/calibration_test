//
// Created by jakub on 27.05.17.
//

#ifndef TESTCV_CALIBDATA_H
#define TESTCV_CALIBDATA_H

#include <opencv2/opencv.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>

class CalibData {
    std::vector<float> rvec;
    std::vector<float> tvec;
    std::vector<cv::Point2f> laser_all;
    std::vector<cv::Point2f> laser_sel;
};


#endif //TESTCV_CALIBDATA_H
