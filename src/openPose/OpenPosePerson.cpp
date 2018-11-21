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

cv::Rect OpenPosePerson::getRect() {
    int min_x = 10000;
    int max_x = 0;
    int min_y = 10000;
    int max_y = 0;
    for (auto itr_coord = _body_parts_coord.begin(); itr_coord != _body_parts_coord.end(); ++itr_coord) {
        int x = itr_coord->x;
        int y = itr_coord->y;
        if (min_x > x)
            if (x > 0)
                min_x = x;
        if (min_y > y)
            if (y > 0)
                min_y = y;
        if (max_x < x)
            max_x = x;
        if (max_y < y)
            max_y = y;
    }

    // マスク領域の指定
    int margin_x_left = 50;
    int margin_x_right = 50;
    int margin_y_up = 50;
    int margin_y_down = 80;
    return cv::Rect(min_x - margin_x_left, min_y - margin_y_up, max_x - min_x + margin_x_right,
                    max_y - min_y + margin_y_down);
}
