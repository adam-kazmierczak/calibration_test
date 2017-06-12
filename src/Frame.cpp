//
// Created by jakub on 24.05.17.
//

#include "../include/Frame.h"


Frame::Frame(const vector<Point3f> &laserPts, const vector<double > &rvec, const vector<double> &tvec) : laserValidPtsXY(
        laserPts), rvec(rvec), tvec(tvec) {
}

const vector<double> &Frame::getTvec() const {
    return tvec;
}

void Frame::calcAngle() {
    angleXoZ = tan( tvec.at(0) / tvec.at(2) );
}

float Frame::getAngleXoZ() const {
    return angleXoZ;
}

Frame::Frame(const vector<double> &_laserPtsRfi) : laserPtsRfi(_laserPtsRfi) {}

void Frame::calcLaserXY(LaserDevice &l) {
if(laserPtsRfi.empty()) {
        std::cout<< "ERROR convValidLaserSets: validLaser vector is empty!" <<std::endl;
    }
    double range = l.angle_max - l.angle_min;
        double theta;
        for( int k = 0; k < laserPtsRfi.size(); ++k) {
            double r =  laserPtsRfi.at(k);
            // it is expected that points that belong to chessboard are within the range
            if(r > 0.01 && r > l.min_board_dist && r < l.max_board_dist) {
                switch (l.scanning_direction) {
                    case LaserDevice::CLOCKWISE:
                        theta = (l.angle_min + range * ((double) k / l.num_points)) * (M_PI / 180.0);
                        break;
                    case LaserDevice::COUNTER_CLOCKWISE:
                        theta = (l.angle_max - range * ((double) k / l.num_points)) * (M_PI / 180.0);
                        break;
                    default:
                        //TODO: manage exceptions
                        std::cout << "WRONG LASER DIRECTION SPECIFICATION!" << std::endl;
                        break;
                }
                //B: consider swapping x and y
                float x = -1.0 * float(r * cos(theta));
                float y = -1.0 * float(r * sin(theta));

                laserValidPtsXY.push_back(Point3f(x,y,1));
//                laserValidPtsXY.push_back(Point3f(x,1,y));
//                laserValidPtsXY.push_back(Point3f(y,x,1));
//                laserValidPtsXY.push_back(Point3f(y,1,x));
//                laserValidPtsXY.push_back(Point3f(1,x,y));
//                laserValidPtsXY.push_back(Point3f(1,y,x));

                laserValidPtsThetas.push_back(float(theta));
            }
       }

    }

void Frame::findLaserBoard() {
    angleXoZ = tan( tvec.at(0) / tvec.at(2) );
    //Find nearest neighbour to chessboard angle
    std::vector<float>::iterator low;
    low = std::lower_bound(laserValidPtsThetas.begin(), laserValidPtsThetas.end(), angleXoZ);
    int nearest_pos = int(low - laserValidPtsThetas.begin());

    //adding points on the left side from vector of center of chessboard
    float allowed_distance = 0.08; // in meters

    int it = nearest_pos;
    while (cv::norm(laserValidPtsXY.at(it) - laserValidPtsXY.at(it-1)) < allowed_distance) {
        cv::Point3f pt = laserValidPtsXY.at(it);
        boardLaserPoints.push_front(pt);
        it--;
        std::cout<< "Push FRONT Current dist is " << cv::norm(laserValidPtsXY.at(it) - laserValidPtsXY.at(it-1)) <<endl;
    }

    it = nearest_pos + 1;
    while (cv::norm(laserValidPtsXY.at(it) - laserValidPtsXY.at(it+1)) < allowed_distance) {
        cv::Point3f pt = laserValidPtsXY.at(it);
        boardLaserPoints.push_back(pt);
        it++;
    }

    std::vector<Point3f> vct;

    for(auto k : boardLaserPoints) {
        vct.push_back(k);
    }

    //find a line equation
    cv::fitLine(vct, lineProps, CV_DIST_L2, 0, 0.01, 0.01);

if(Frame::verbose) {
        std::cout<< "Frame: calcLaserXY finished" <<std::endl;
    }
}

void Frame::setRvec(const vector<double> &rvec) {
    Frame::rvec = rvec;
//    rotMatcv;
//    Mat tempM = Mat::zeros(1,3,CV_32FC1);
//    tempM.at(0) =
    cv::Rodrigues(rvec, rotMatcv);
//    Eigen::Matrix<float,3,3> ;
    cv2eigen(rotMatcv,rotMatE);
    //B: added
//    rotMatE *= 1.0;
    cout << " Making frame! Size of rotMatcv is " <<rotMatcv.size() <<endl;
    cout << " CV matrix is " <<endl << rotMatcv <<endl;
    cout << " Eigen matrix is " <<endl << rotMatE <<endl;
    cout << " Eigen 3rd row of matrix is " <<endl << rotMatE.col(2) <<endl;
}

void Frame::setTvec(const vector<double> &tvec) {
    Frame::tvec = tvec;

//    double* ptr = &this->tvec[0];
//    Eigen::Map<Eigen::Vector3d> tvecE(ptr, 3);
//    this->tvecE = tvecE;
    //B: modified to -1
    tvecE[0] = tvec[0];
    tvecE[1] = tvec[1];
    tvecE[2] = tvec[2];
    cout << " Eigen trans matrix is " <<endl << this->tvecE <<endl;
//    int t;
//    cin >> t;
}

void Frame::draw(cv::Mat &screen) {
//    Point2f offs(300,300);
    int offs = 300;
    int pix_per_m = 100;
    for(auto it : boardLaserPoints) {
        Point pt;
        pt.x = int((it.x) * (float)pix_per_m) +offs;
        pt.y = int((it.y) * (float)pix_per_m) +offs;
        cv::line(screen, pt, pt, cv::Scalar(0,0,255), 1, 8);
    }

    float sc_factor = 2.0;
    //draw line orientation lineProps is ordered in: vector[x, y] point[x, y]
// Original
//    cv::Point center(int(lineProps.val[2]*float(pix_per_m))+offs, int(lineProps.val[3]*float(pix_per_m))+offs);
//    cv::Point begin(center.x - int(lineProps.val[0]*float(pix_per_m)*sc_factor), center.y - int(lineProps.val[1]*float(pix_per_m) * sc_factor));
//    cv::Point end(center.x + int(lineProps.val[0]*float(pix_per_m)*sc_factor), center.y + int(lineProps.val[1]*float(pix_per_m) * sc_factor));
//    cv::line(screen, begin, end, cv::Scalar(0,255,0), 1, 8);

    // indexes 2,3 changed to 3,4
    cv::Point center(int(lineProps.val[3]*float(pix_per_m))+offs, int(lineProps.val[4]*float(pix_per_m))+offs);
    cv::Point begin(center.x - int(lineProps.val[0]*float(pix_per_m)*sc_factor), center.y - int(lineProps.val[1]*float(pix_per_m) * sc_factor));
    cv::Point end(center.x + int(lineProps.val[0]*float(pix_per_m)*sc_factor), center.y + int(lineProps.val[1]*float(pix_per_m) * sc_factor));
    cv::line(screen, begin, end, cv::Scalar(0,255,0), 1, 8);
}

void Frame::calcEQandN(Equation &eq) {
    this->N = -1.0 * rotMatE.col(2) * (rotMatE.col(2).transpose() * (this->tvecE));
    cout << "N is calcutated and it is: " << N << endl;
    cout << "N squared norm is: " << N.squaredNorm() << endl;
    cout << "Size of boardPoints: " << boardLaserPoints.size() << endl;

    for(auto it : boardLaserPoints) {
        std::cout << "Pushing stuff to eq" <<std::endl;
        eq.pushPointEQ(it, N);
    }
    //Make A vector for a point
    // Use this for solving = https://eigen.tuxfamily.org/dox/group__LeastSquares.html
}
