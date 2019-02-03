//
// Created by yagi on 18/07/25.
//

#ifndef SFMDR_FOOTPRINT_OPENPOSEPERSON_H
#define SFMDR_FOOTPRINT_OPENPOSEPERSON_H

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

class OpenPosePerson {
public:
    OpenPosePerson() {};
    ~OpenPosePerson() {};

    void setBodyCoord(std::vector<std::string> coord);
    std::vector<cv::Point2f> getBodyCoord();
    cv::Rect getRect();
    void clearBodyCoord();
    int humanID = 0;

    std::vector<cv::Point2f> _body_parts_coord;
    std::vector<float> _probabilityList;
    cv::Rect humanRect;
    cv::Point2f rFoot;
    cv::Point2f lFoot;
};


#endif //SFMDR_FOOTPRINT_OPENPOSEPERSON_H
