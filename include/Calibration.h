//
// Created by jakub on 24.05.17.
//

#ifndef TESTCV_CALIBRATION_H
#define TESTCV_CALIBRATION_H
#include "../include/Frame.h"
#include "../include/Equation.h"
#include "../tinyXML/tinyxml2.h"
#include <Eigen/Dense>
class Frame;
struct MyPoint {
    cv::Point2f pt;
    double r;

    MyPoint(const Point2f &pt, double r) : pt(pt), r(r) {}
};

class LaserDevice {
public:
    static constexpr int CLOCKWISE = 1;
    static constexpr int COUNTER_CLOCKWISE = 2;

    double angle_min;
    double angle_max;
    double num_points;
    double resolution;
    int ignored_records;
    int scanning_direction;

    double min_board_dist;
    double max_board_dist;

public:
    LaserDevice();
    LaserDevice(const std::string filename);
//    void readConfig(const std::string filename);
};


class Calibration {
    bool stopAtImage = false;
    LaserDevice laser;
    std::vector<string> framesNames;
    std::vector<int> validIDs;
    vector<vector<double>> tempLaser;
    vector<vector<double>> validLaser;
    vector<vector<cv::Point2f>> validLaserXY;
    vector<vector<float>> validThetas;
    vector<double> rawImgVectors;
    float board_width_mm;
    float board_height_mm;

    bool verbose = true;
    std::vector<Frame> frames;
    // size of chessboard two ints
    // chessboard raster size
    // cheesboard points coordinate
    ///Results: translation and rotation

    ///functions:
    //Config
    //Load Frames
    //
public:
    Equation eq;

    Calibration(const std::string filename);

    /// Functions related with FRAMES DATA
    void loadFramesData(const std::string cameraXML_out, const std::string cameraXML_config);
    void orderFramesData();

    /// Functions related with LASER DATA
    void attachTXTData(const std::string validFramesIDsFile, const std::string laserTXT, const std::string cameraXML);
    void readLaserSetsTXT(const std::string filename);
    void readValidIDs(const std::string validFramesIDsFile);
    void pickValidLaserSets();
    void convValidLaserSets(); //convert spaces from [alpha, radius] to [x, y]
    void wrapDataToItems();
    void printValidLaserXY();
    void printCalibrationData();
};


#endif //TESTCV_CALIBRATION_H
