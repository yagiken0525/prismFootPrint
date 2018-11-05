//
// Created by yagi on 18/10/07.
//

#ifndef SFMDR_FOOTPRINT_K_MEANS_H
#define SFMDR_FOOTPRINT_K_MEANS_H

#include "iostream"
#include "opencv2/opencv.hpp"

class k_means {
public:
    void clustering(std::vector<cv::Point3f> points, int k);
};


#endif //SFMDR_FOOTPRINT_K_MEANS_H
