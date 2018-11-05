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
    int loadCameraRt(std::string file_name);

    void setR(std::vector<float>  R){
        _R = R;
    }
    void setT(cv::Point3f T){
        _T = T;
    }
    void setId(int id){
        _id = id;
    }
    std::vector<float> getR(){
        return _R;
    }
    cv::Point3f getT(){
        return _T;
    }
//    cv::Mat getT(){
//        return _T;
//    }
    int getId(){
        return _id;
    }
    int _id;
    std::vector<float> _R;
    cv::Mat R_inv;
    cv::Point3f _T;
//    cv::Mat _R = cv::Mat::zeros(3, 3, CV_64F);
//    cv::Mat _T = cv::Mat::zeros(1, 3, CV_64F);


private:

};


#endif //SFMDR_FOOTPRINT_CAMERA_H
