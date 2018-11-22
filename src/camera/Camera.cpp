//
// Created by yagi on 18/07/24.
//

#include "Camera.h"
#include "../basicFunction/basicFunction.h"

using namespace std;

int Camera::loadCameraA(std::string file_name){
    ifstream ifs(file_name);
    string buf;

    if(!ifs.is_open()){
        std::cout << "cannot open "<< file_name << std::endl;
        return false;
    }
    std::cout << "correctly opened "<< file_name << std::endl;

    while(getline(ifs, buf)){
        vector<string> wordsInLine = yagi::split(buf, ' ');
        if (wordsInLine[0] == "FocalLength:"){
            focalPoint.x = stof(wordsInLine[1]);
            focalPoint.y = stof(wordsInLine[2]);
        }else if(wordsInLine[0] == "PrincipalPoint:") {
            centerPoint.x = stof(wordsInLine[1]);
            centerPoint.y = stof(wordsInLine[2]);
        }else if(wordsInLine[0] == "RadialDistortion:") {
            radialDistortion.x = stof(wordsInLine[1]);
            radialDistortion.y = stof(wordsInLine[2]);
        }else if(wordsInLine[0] == "TangentialDistortion:") {
            translateDistortion.x = stof(wordsInLine[1]);
            translateDistortion.y = stof(wordsInLine[2]);
        }
    }

//    this->_A.at<double>(0,0) = focalPoint.x;
//    this->_A.at<double>(1,1) = focalPoint.y;
//    this->_A.at<double>(0,2) = centerPoint.x;
//    this->_A.at<double>(1,2) = centerPoint.y;
//    this->_A.at<double>(2,2) = 1.0;

    this->_A.at<double>(0,0) = focalPoint.x;
    this->_A.at<double>(1,1) = focalPoint.y;
    this->_A.at<double>(0,2) = centerPoint.x;
    this->_A.at<double>(1,2) = centerPoint.y;
    this->_A.at<double>(2,2) = 1.0;

    this->_dist.at<double>(0,0) = radialDistortion.x;
    this->_dist.at<double>(0,1) = radialDistortion.y;
    this->_dist.at<double>(0,2) = translateDistortion.x;
    this->_dist.at<double>(0,3) = translateDistortion.y;

    return 1;
}

int Camera::loadCameraRt(std::string file_name){
    ifstream ifs(file_name);
    string buf;

    if(!ifs.is_open()){
        std::cout << "cannot open "<< file_name << std::endl;
        return false;
    }
    cout << "Correctly open" << endl;

    cv::Point3d R;
    cv::Point3d T;

    while(std::getline(ifs,buf)){
        vector<string> wordsInLine = yagi::split(buf, ' ');
        if (wordsInLine[0] == "Rotation:"){
            R.x = stof(wordsInLine[1]);
            R.y = stof(wordsInLine[2]);
            R.z = stof(wordsInLine[3]);
        }else if(wordsInLine[0] == "Translation:") {
            T.x = stof(wordsInLine[1]);
            T.y = stof(wordsInLine[2]);
            T.z = stof(wordsInLine[3]);
        }
    }
    this->_R.at<double>(0,0) = R.x;
    this->_R.at<double>(0,1) = R.y;
    this->_R.at<double>(0,2) = R.z;
    this->_T.at<double>(0,0) = T.x;
    this->_T.at<double>(0,1) = T.y;
    this->_T.at<double>(0,2) = T.z;
    this->_Tvec = T;
    this->_Rvec = R;

    cv::Rodrigues(this->_Rvec, this->_Rmat);
    this->_Rinv = this->_Rmat.inv();


    return 1;
}

int Camera::loadCameraP(std::string file_name){
    ifstream ifs(file_name);
    string buf;

    if(!ifs.is_open()){
        std::cout << "cannot open "<< file_name << std::endl;
        return false;
    }

    cv::Point2f focalPoint;
    cv::Point2f centerPoint;

    while(ifs >> buf){
        std::getline(ifs,buf);
        vector<string> wordsInLine = yagi::split(buf, ' ');
        if (wordsInLine[0] == "FocalLength:"){
            focalPoint.x = stof(wordsInLine[1]);
            focalPoint.y = stof(wordsInLine[2]);
        }else if(wordsInLine[0] == "PrincipalPoint:") {
            centerPoint.x = stof(wordsInLine[1]);
            centerPoint.y = stof(wordsInLine[2]);
        }
    }
    this->_A.at<double>(0,0) = focalPoint.x;
    this->_A.at<double>(1,1) = focalPoint.y;
    this->_A.at<double>(0,2) = centerPoint.x;
    this->_A.at<double>(1,2) = centerPoint.y;
    this->_A.at<double>(2,2) = 1.0;

    return 1;
}


int Camera::outputRt(std::string file_name) {
    ofstream outputfile(file_name);

    outputfile << "Rotation: "
               << this->getR().at<double>(0, 0) << " "
               << this->getR().at<double>(0, 1) << " "
               << this->getR().at<double>(0, 2) << endl;
    outputfile << "Translation: "
               << this->getT().at<double>(0, 0) << " "
               << this->getT().at<double>(0, 1) << " "
               << this->getT().at<double>(0, 2) << endl;
    outputfile.close();
}


int Camera::outputP(std::string file_name) {
    ofstream outputfile(file_name);
    outputfile << this->getP() << endl;
    outputfile.close();
}