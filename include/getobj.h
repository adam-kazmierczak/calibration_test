#ifndef GETOBJ_H
#define GETOBJ_H

#include <iostream>

#include <opencv2/opencv.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

void getObj(Mat frame, Mat back);

#endif // GETOBJ_H
