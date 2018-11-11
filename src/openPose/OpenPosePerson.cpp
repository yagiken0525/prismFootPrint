//
// Created by yagi on 18/07/25.
//

#include "OpenPosePerson.h"

using namespace std;

void OpenPosePerson::setBodyCoord(vector<string> coord) {
    cv::Point2f coord_f(stof(coord[0]), stof(coord[1]));
    float prob(stof(coord[2]));
    _body_parts_coord.push_back(coord_f);
    _probabilityList.push_back(prob);
}


vector<cv::Point2f> OpenPosePerson::getBodyCoord() {
    return this->_body_parts_coord;
}


void OpenPosePerson::clearBodyCoord() {
    _body_parts_coord.clear();
    _probabilityList.clear();
}