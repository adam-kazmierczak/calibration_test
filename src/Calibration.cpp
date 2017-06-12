//
// Created by jakub on 24.05.17.
//

#include "../include/Calibration.h"

LaserDevice::LaserDevice(const std::string filename) {
    FileStorage fsCamConf(filename, FileStorage::READ);
    FileNode n = fsCamConf["Laser"];
    n["Laser_Angle_Min"] >> angle_min;
    n["Laser_Angle_Max"] >> angle_max;
    n["Laser_Direction"] >> scanning_direction;
    n["Laser_Num_Points"] >> num_points;
    n["Laser_Ignored_Records"] >> ignored_records;
    n["Laser_Min_Board_Distance"] >> min_board_dist;
    n["Laser_Max_Board_Distance"] >> max_board_dist;
    fsCamConf.release();
    double range = angle_max - angle_min;
    resolution = range / num_points;
    if(true){
        std::cout << " *** Laser Device ***"<< std::endl;
        std::cout << "Laser angular resolution is: " << resolution << " deg" << std::endl;
        std::cout << "Laser angular range: [" << angle_min << ", " << angle_max << "]" << std::endl;
    }
}

LaserDevice::LaserDevice() {}

Calibration::Calibration(const std::string filename) : laser(filename){
//    cv::FileStorage fs("../xml/SourceCalibIMGs.xml", FileStorage::READ);
//    cv::FileNode n = fs["Settings"];
//    std::string calibIMGs_xml;
//    n["Input"] >> calibIMGs_xml;
//    fs.release();
//    cv::FileStorage fsIMGnames(calibIMGs_xml, FileStorage::READ);
//    fsIMGnames["images"] >> framesNames;
//    fsIMGnames.release();
}

void Calibration::attachTXTData(const std::string validFramesIDsFile, const std::string laserTXT,
                                const std::string cameraXML) {
}

//Q: is it better to keep trash data in a field of class or just pass it as an agument to a function?
//void Calibration::readLaserSetsTXT(const std::string filename, vector<vector<double>> &tempLaser) {
    void Calibration::readLaserSetsTXT(const std::string filename) {
    std::string line;
    std::ifstream infile(filename); // relative directory depends on the build path
    if(infile.fail()) {
        std::cout << "readLaserSetsTXT: error loading .txt file" <<std::endl;
    }
    while (std::getline(infile, line)) {
        std::vector<double> row;
        std::istringstream iss(line);
        std::string val_string;
        int cnt = 0;

        while (iss >> val_string) {
            //std::cout << "Value read " << val_string <<std::endl;
            //std::cout << "cnt vs num_ignored " << cnt << " " << num_ignored << std::endl;
            if(cnt++ >= laser.ignored_records) {
                if(val_string == "inf," || val_string == "nan,") {
                    //push zero to keep angular increments order (zero will be ignored in next steps)
                    row.push_back(0.f);
                    continue;
                }
                double val = ::atof(val_string.c_str());
                row.push_back(val);
            }else { /*std::cout << "Value was ignored" <<std::endl;*/}
        }
        tempLaser.push_back(row);
    }
    if(verbose) {
        cout << "*** readLaserSetsTXT ***" << endl;
        cout << "TempLaser Size: " << tempLaser.size() << endl;
//        cout << "TempLaser Size at 61: " << tempLaser.at(60).size() << endl<< endl;
    }
}

void Calibration::readValidIDs(const std::string validFramesIDsFile) {
    // Get numbers of frames where chessboard was found
    FileStorage fsRead(validFramesIDsFile, FileStorage::READ);
    fsRead["validFramesIDs"] >> validIDs;
    fsRead.release();
    if(verbose) { std::cout << "*** readValidIDs *** size of ValidIDs is: " << validIDs.size() << std::endl; }
}

void Calibration::pickValidLaserSets() {
    for(auto it : validIDs) {
        //crop ignored values at the start of each scan
        vector<double>::const_iterator first = tempLaser.at(it).begin(); //currently not in use: "+ num_ignored;"
        vector<double>::const_iterator last = tempLaser.at(it).end();
        vector<double> tmp_laserscan(first, last);
        //push back valid subvector to vector of vectors
        validLaser.push_back(tmp_laserscan);
        // Test creating frame objects here
//        Frame fr(tmp_laserscan);

        cout << "readValidLaserSets: Current subvect size " << tmp_laserscan.size() << endl;
//        cout << "Current it: " << it << endl;
    }
}

void Calibration::loadFramesData(const std::string cameraXML_out, const std::string cameraXML_config) {
    FileStorage fsCamConf(cameraXML_config, FileStorage::READ);
    fsCamConf["Board_WidthMM"] >> board_width_mm;
    fsCamConf["Board_HeightMM"] >> board_height_mm;

    FileStorage fsCamOut(cameraXML_out, FileStorage::READ);
    FileNode n = fsCamOut["Extrinsic_Parameters"];  // Read Structure sequence - Get node
    n["data"] >> rawImgVectors;
    fsCamOut.release();
    if(verbose) { std::cout << "*** loadFramesData *** size is: " << rawImgVectors.size() << std::endl; }
}

void Calibration::printCalibrationData() {

}

void Calibration::convValidLaserSets() {
    if(validLaser.empty()) {
        std::cout<< "ERROR convValidLaserSets: validLaser vector is empty!" <<std::endl;
    }
    double range = laser.angle_max - laser.angle_min;
    for(int scan = 0; scan < validLaser.size(); ++scan) {
        std::vector<cv::Point2f> scanXY;
        std::vector<float> scanThetas;
        double theta;
        for( int k = 0; k < validLaser.at(scan).size(); ++k) {
            double r =  validLaser.at(scan).at(k);
            //TODO: implement counter-clockwise data order
            // it is expected that points that belong to chessboard are within the range
            if(r > 0.01 && r > laser.min_board_dist && r < laser.max_board_dist) {
                switch (laser.scanning_direction) {
                    case LaserDevice::CLOCKWISE:
                        theta = (laser.angle_min + range * ((double) k / laser.num_points)) * (M_PI / 180.0);
                        break;
                    case LaserDevice::COUNTER_CLOCKWISE:
                        theta = (laser.angle_max - range * ((double) k / laser.num_points)) * (M_PI / 180.0);
                        break;
                    default:
                        //TODO: manage exceptions
                        std::cout << "WRONG LASER DIRECTION SPECIFICATION!" << std::endl;
                        break;
                }
                float x = float(r * cos(theta));
                float y = float(r * sin(theta));

                scanXY.push_back(Point2f(x,y));
                scanThetas.push_back(float(theta));
            }
       }
        validLaserXY.push_back(scanXY);
        validThetas.push_back(scanThetas);
    }
    if(verbose) {
        std::cout<< "convValidLaserSets finished." <<std::endl;
    }
}

void Calibration::printValidLaserXY() {
    cv::namedWindow("LaserScan", 1);
    cv::namedWindow("Current_Frame", 1);
    //loadIMG data
    int resolution_x = 600;
    int resolution_y = 600;
    int pix_per_m = 100;
    int i = 0;
    for(auto it : validLaserXY) {
        std::cout << "Current IMG ID: " << validIDs.at(i) << std::endl;
//        cv::Mat current_frame = imread(framesNames.at(val))
        cv::Mat black = Mat::zeros(resolution_y, resolution_x, CV_8UC3);
        vector<float> curr_thetas = validThetas[i];
        std::cout << "curr_thetas size = " << curr_thetas.size() << std::endl;

        //Find nearest neighbour to chessboard angle
        std::vector<float>::iterator low;
        low = std::lower_bound(curr_thetas.begin(), curr_thetas.end(), frames.at(i).getAngleXoZ());
        int nearest_pos = int(low - curr_thetas.begin());

        int j =0;
        for (auto k : it) {
            cv::Point pt;
            pt.x = int((k.x) * (float)pix_per_m) + 300;
            pt.y = int((k.y) * (float)pix_per_m) + 300;
            Scalar color(111, 122, 133);
//            if( j < 50) {color = Scalar (255,255,255);}
//            if( j > (nearest_pos - 10) && j < (nearest_pos)) { color = Scalar(0,0,255);}
            cv::line(black, pt, pt, color, 1, 8);
            j++;
        }

        //Note that Z translation in OpenCV camera coordinate system is trated as X coordinate in Laser coordinate system
        int estimated_transl_y = int(frames.at(i).getTvec().at(0) + 300);
        int estimated_transl_z = int(frames.at(i).getTvec().at(2) + 300);
        cv::line(black, cv::Point(300,300), cv::Point(estimated_transl_z, estimated_transl_y), cv::Scalar(255,255,255), 2, 8);
        //print coordinate X, Y axis
        cv::line(black, cv::Point(300,300), cv::Point(300 + 50, 300), cv::Scalar(0,0,255), 2, 8);
        cv::line(black, cv::Point(300,300), cv::Point(300, 300 + 50), cv::Scalar(255 ,0,255), 2, 8);
        frames.at(i).draw(black);
//        cv::flip(black, black, 0);
        cv::imshow("LaserScan", black);
        if(stopAtImage)
            cv::waitKey(0);
        i++;
    }
}

void Calibration::wrapDataToItems() {
    for(int i = 0; i < validIDs.size(); ++i) {
        //Add rotation and translation vector to each Frame
        vector<double>::const_iterator start = rawImgVectors.begin() + i * 6; //currently not in use: "+ num_ignored;"
        vector<double> tmp_rvec(start, start +3);
        vector<double> tmp_tvec(start + 3, start + 6);
        vector<Point2f> tmp_scanXY(validLaserXY.at(i).begin(), validLaserXY.at(i).end());
//        Frame fr(tmp_scanXY, tmp_rvec, tmp_tvec);
        Frame fr(validLaser.at(i));
        fr.calcLaserXY(laser);

        //B: validation added
        for(int it = 0; it < 3; ++it) {
            if( abs(tmp_rvec.at(it)) > M_PI){
                if(tmp_rvec.at(it) > 0.0)
                    tmp_rvec.at(it) -= M_PI;
                else
                    tmp_rvec.at(it) += M_PI;
            }
        }

        fr.setRvec(tmp_rvec);
        fr.setTvec(tmp_tvec);

//        fr.calcAngle();
        fr.findLaserBoard();
        fr.calcEQandN(eq);

        frames.push_back(fr);
    }
}

