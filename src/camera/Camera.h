//
// Created by yagi on 18/07/24.
//

#ifndef SFMDR_FOOTPRINT_CAMERA_H
#define SFMDR_FOOTPRINT_CAMERA_H

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "../openPose/OpenPosePerson.h"

class Camera {
public:
    Camera(){
        _R = cv::Mat::zeros(3,3,CV_64F);
        _T = cv::Mat::zeros(3,3,CV_64F);
        _A = cv::Mat::zeros(3,3,CV_64F);
        _P = cv::Mat::zeros(3,4,CV_64F);
        _dist = cv::Mat::zeros(1,4,CV_64F);
    };
    int loadCameraA(std::string file_name);
    int loadCameraRt(std::string file_name);
    int loadCameraP(std::string file_name);
    int outputRt(std::string file_name);
    int outputP(std::string file_name);

    void setR(cv::Mat R){ _R = R; }
    void setT(cv::Mat T){ _T = T; }
    void setA(cv::Mat A){ _A = A; }
    void setP(cv::Mat P){ _P = P; }
    void setDist(cv::Mat dist){ _dist = dist; }
    cv::Mat getR(){ return _R; }
    cv::Mat getT(){ return _T; }
    cv::Mat getA(){ return _A; }
    cv::Mat getP(){ return _P; }
    cv::Mat getDist(){ return _dist; }

    cv::Point2d focalPoint;
    cv::Point2d centerPoint;
    cv::Point2d radialDistortion;
    cv::Point2d translateDistortion;

    cv::Mat _R = cv::Mat::zeros(3,3,CV_64F);
    cv::Mat _T = cv::Mat::zeros(3,3,CV_64F);
    cv::Mat _A = cv::Mat::zeros(3,3,CV_64F);
    cv::Mat _P = cv::Mat::zeros(3,4,CV_64F);
    cv::Mat _dist = cv::Mat::zeros(1,4,CV_64F);
    cv::Mat _Rmat = cv::Mat::zeros(3,3,CV_64F);
    cv::Mat _Rinv = cv::Mat::zeros(3,3,CV_64F);
    cv::Vec3d _Tvec;
    cv::Vec3d _Rvec;

};


#endif //SFMDR_FOOTPRINT_CAMERA_H
