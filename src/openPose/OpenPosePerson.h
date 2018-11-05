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
    void clearBodyCoord();
    int humanID = 0;
private:
    std::vector<cv::Point2f> _body_parts_coord;
};


#endif //SFMDR_FOOTPRINT_OPENPOSEPERSON_H
