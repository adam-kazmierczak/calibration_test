//
// Created by jakub on 09.06.17.
//

#ifndef TESTCV_EQUATION_H
#define TESTCV_EQUATION_H
#include <opencv2/opencv.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <cmath>

    using namespace std;
    using namespace cv;

struct Equation {

    Equation() {}
    void pushPointEQ(cv:: Point3f pt, Eigen::Vector3d N) {
        Eigen::Matrix<double, 1, 9> partA;
//        partA(0,0) = pt.x * N(0);
//        partA(0,1) = pt.y * N(0);
//        partA(0,2) = N(0);
//        partA(0,3) = pt.x * N(1);
//        partA(0,4) = pt.y * N(1);
//        partA(0,5) = N(1);
//        partA(0,6) = pt.x * N(2);
//        partA(0,7) = pt.y * N(2);
//        partA(0,8) = N(2);

//        123 132 213 231 312 321
//        0   0   3   3   6   6     0   0
//        1   1   4   4   7   7     1   1
//        2   2   5   5   8   8     2   2

//        3   6   0   6   0   3     3   3
//        4   7   1   7   1   4     4   4
//        5   8   2   8   2   5     5   5

//        6   3   6   0   3   0     6   6
//        7   4   7   1   4   1     7   7
//        8   5   8   2   5   2     8   8

//        132
//        partA(0,0) = pt.x * N(0);
//        partA(0,2) = pt.y * N(0);
//        partA(0,1) = N(0);
//        partA(0,3) = pt.x * N(1);
//        partA(0,5) = pt.y * N(1);
//        partA(0,4) = N(1);
//        partA(0,6) = pt.x * N(2);
//        partA(0,8) = pt.y * N(2);
//        partA(0,7) = N(2);


////        213
//        partA(0,1) = pt.x * N(0);
//        partA(0,0) = pt.y * N(0);
//        partA(0,2) = N(0);
//        partA(0,4) = pt.x * N(1);
//        partA(0,3) = pt.y * N(1);
//        partA(0,5) = N(1);
//        partA(0,7) = pt.x * N(2);
//        partA(0,6) = pt.y * N(2);
//        partA(0,8) = N(2);

//        231
//        partA(0,1) = pt.x * N(0);
//        partA(0,2) = pt.y * N(0);
//        partA(0,0) = N(0);
//        partA(0,4) = pt.x * N(1);
//        partA(0,5) = pt.y * N(1);
//        partA(0,3) = N(1);
//        partA(0,7) = pt.x * N(2);
//        partA(0,8) = pt.y * N(2);
//        partA(0,6) = N(2);
//
//
////        312
//        partA(0,2) = pt.x * N(0);
//        partA(0,0) = pt.y * N(0);
//        partA(0,1) = N(0);
//        partA(0,5) = pt.x * N(1);
//        partA(0,3) = pt.y * N(1);
//        partA(0,4) = N(1);
//        partA(0,8) = pt.x * N(2);
//        partA(0,6) = pt.y * N(2);
//        partA(0,7) = N(2);
//
////        321
//        partA(0,2) = pt.x * N(0);
//        partA(0,1) = pt.y * N(0);
//        partA(0,0) = N(0);
//        partA(0,5) = pt.x * N(1);
//        partA(0,4) = pt.y * N(1);
//        partA(0,3) = N(1);
//        partA(0,8) = pt.x * N(2);
//        partA(0,7) = pt.y * N(2);
//        partA(0,6) = N(2);
        //123
        partA(0,0) = pt.x * N(0);
        partA(0,1) = pt.y * N(0);
        partA(0,2) = pt.z * N(0);
        partA(0,3) = pt.x * N(1);
        partA(0,4) = pt.y * N(1);
        partA(0,5) = pt.z * N(1);
        partA(0,6) = pt.x * N(2);
        partA(0,7) = pt.y * N(2);
        partA(0,8) = pt.z * N(2);

        cont_A.push_back(partA);
        cont_B.push_back(N.squaredNorm());
    }

    void finCalc() {
        std::cout << "cont_A.size()" << cont_A.size();
        std::cout << "cont_B.size()" << cont_B.size();
//        Eigen::Matrix<double, 1, 9> A;
//        Eigen::Matrix<double, cont_A.size(), 9> A;
        Eigen::MatrixXd A(cont_A.size(), 9);
//        A.resize(cont_A.size(), 9);
//        Eigen::Matrix<double, 1, 1> B;
        Eigen::MatrixXd B(cont_B.size(), 1);
//        B.resize(cont_B.size(), 1);

        for(int i = 0; i < cont_A.size(); ++i) {
            A.block(i,0,1,9) = cont_A.at(i);
            B(i,0) = cont_B.at(i);
        }
        X = A.colPivHouseholderQr().solve(B);
        cout << " Solution is: " << X << endl;
        Eigen::MatrixXd H(3,3);
//        for(int i = 0; i <9; ++i) {
//            int id_col =
//            H(0,0) = X(0,0);
//        }


//        123 132 213 231 312 321
//        0   0   3   3   6   6     0   0
//        1   1   4   4   7   7     1   1
//        2   2   5   5   8   8     2   2
//        3   6   0   6   0   3     3   3
//        4   7   1   7   1   4     4   4
//        5   8   2   8   2   5     5   5
//        6   3   6   0   3   0     6   6
//        7   4   7   1   4   1     7   7
//        8   5   8   2   5   2     8   8

        H(0,0) = X(0,0);
        H(0,1) = X(1,0);
        H(0,2) = X(2,0);
        H(1,0) = X(3,0);
        H(1,1) = X(4,0);
        H(1,2) = X(5,0);
        H(2,0) = X(6,0);
        H(2,1) = X(7,0);
        H(2,2) = X(8,0);


        cout << " This is H : " << H << endl;

        Eigen::MatrixXd L(3,3);

        L.col(0) = H.col(0);

        Eigen::Vector3d V1(H.col(0));
        Eigen::Vector3d V2(H.col(1));

        L.col(1) = ((-1.0) * V1).cross(V2);
        L.col(2) = H.col(1);

//        cout << " *** Test on vectors *** " <<endl;
//        cout << V1.cross(V2) << endl;

        Eigen::MatrixXd T(3,1);
        Eigen::Matrix<double, 3, 3> R;
        T = ((-1.0) * L.transpose()) * H.col(2);
        cout << " Translate is: " << endl<< T << endl;
        //Rotation
        R = L.transpose();
        Eigen::Vector3d eu;
        eu = R.eulerAngles(0,1,2);
        cout << " Rotate mat is: " << endl<< R << endl;
        cout << " Euler angles are: " << endl<< eu  * (180.0/3.1415) << endl;
// * (180.0/3.1415)
    }

    std::vector<Eigen::Matrix<double , 1, 9>> cont_A;
    Eigen::Matrix<double, 9, 1> X;
    std::vector<double> cont_B;
};


#endif //TESTCV_EQUATION_H
