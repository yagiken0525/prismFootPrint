//
// Created by yagi on 18/01/10.
//

#ifndef MAINTEST_BASICFUNCTION_H
#define MAINTEST_BASICFUNCTION_H

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

namespace yagi {

    std::string digitString(int num, int digit);

    std::vector<std::string> split(const std::string &s, char delim);


    cv::Mat maskAofB(cv::Mat A, cv::Mat B);


    void getGradSegment(cv::Point2f pt1, cv::Point2f pt2, float *grad, float *segment);


    void drawLine(cv::Mat edgeMask, cv::Point2f pt1, cv::Point2f pt2, int thickness, cv::Scalar color);


    void MatToVec(const cv::Mat &src, std::vector<std::vector<unsigned char>> &dst);


    void VecToMat(const std::vector<std::vector<unsigned char>> src, cv::Mat &dst);


    float distPoint2Line(cv::Point2f pt, float a, float b);

    void setColor(std::vector<cv::Scalar> *colors);

    template<typename Point>
    float calc2PointDistance(Point p1, Point p2);

    void mycalcWarpedPoint(std::vector<cv::Point2f> next, std::vector<cv::Point2f> *warped, cv::Mat H);

    void mycalcfloatWarpedPoint(std::vector<cv::Point2f> next, std::vector<cv::Point2f> *warped, cv::Mat H);

    template<typename Point>
    float calcDistance(Point p1, Point p2);




}

#endif //MAINTEST_BASICFUNCTION_H
