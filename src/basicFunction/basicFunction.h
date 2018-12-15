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

    cv::Point2f warpPoint(cv::Point2f srcPt, cv::Mat H);

    template<typename Point>
    float calcDistance(Point p1, Point p2);

    void loadImage(std::string imagePath, std::vector<cv::Mat>* imageList);

    void generatePointClouds(std::vector<cv::Point3f>& objectCorners, int H, int W, float SCALE, int Wstart = 0, int Hstart = 0);


    void generatePointCloudsIn2Dscale(std::vector<cv::Point2f>& objectCorners, int H, int W, float SCALE, const int  AREA_W, const int AREA_H);

    cv::Mat3b generatePointCloudsAsMatrix(const int width);

    void generatePointCloudsAsBlocks(std::vector<cv::Point3f>& objectCorners, int H, int W, float SCALE, int Wstart = 0, int Hstart = 0, int BlockCol = 1, int BlockRow = 1);

    void some_filter(const cv::Mat& in, cv::Mat& out);

    void print_info(const cv::Mat& mat);

    template<typename type>
    void vectorSum(std::vector<type> vec);

    void clickPoints(cv::Mat image, std::vector<cv::Point2f> & clickedPoints, const std::string filePath );

    void push4PointsToVector(std::vector<cv::Point2f> &points, cv::Point2f Hb_pt1, cv::Point2f Hb_pt2, cv::Point2f Hb_pt3, cv::Point2f Hb_pt4);


    void push4_3DPointsToVector(std::vector<cv::Point3f> &points, cv::Point3f Hb_pt1, cv::Point3f Hb_pt2, cv::Point3f Hb_pt3, cv::Point3f Hb_pt4);


    cv::Rect obtainRectFrom4Points(std::vector<cv::Point2f> &points);
}

#endif //MAINTEST_BASICFUNCTION_H
