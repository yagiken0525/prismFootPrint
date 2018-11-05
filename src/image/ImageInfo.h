//
// Created by yagi on 18/07/25.
//

#ifndef SFMDR_FOOTPRINT_IMAGEINFO_H
#define SFMDR_FOOTPRINT_IMAGEINFO_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include "../camera/Camera.h"
#include "../openPose/OpenPosePerson.h"

class ImageInfo {
public:
    cv::Mat image;
    Camera camera;
    std::vector<OpenPosePerson> persons;
};


#endif //SFMDR_FOOTPRINT_IMAGEINFO_H
