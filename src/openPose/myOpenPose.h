//
// Created by yagi on 18/10/17.
//

#ifndef RUNNERSTEPS_MYOPENPOSE_H
#define RUNNERSTEPS_MYOPENPOSE_H

//#include <openpose/flags.hpp>
// OpenPose dependencies
#include <openpose/headers.hpp>

namespace yagi{
    void outputTextFromVideo(const std::string video_path, const std::string output_path);
    void outputTextFromImage(const std::string video_path, const std::string output_path, std::vector<cv::Mat> iamges);
};
#endif //RUNNERSTEPS_MYOPENPOSE_H
