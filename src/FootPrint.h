//
// Created by yagi on 18/07/24.
//

#ifndef SFMDR_FOOTPRINT_FOOTPRINT_H
#define SFMDR_FOOTPRINT_FOOTPRINT_H

#include "pointCloud/Model.h"
#include "image/ImageInfo.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

struct Vote{
    cv::Vec3f vertice;
    std::vector<int> frameNum;
};

class FootPrint {
public:
    FootPrint(std::string project_name){
        _project_name = project_name;
        _data_path = "../Data/";
        _projects_path = _data_path + "Projects/" + _project_name + "/";
        _sfm_projects_path = "/home/yagi/sfmDR/projects/" + _project_name + "/";
    };
    ~FootPrint(){};

    //しきい値
    float DIST_RANGE = 30; // 画像投影点の内どれくらいの距離の点を接地点とみなすか
    int VOTE_RANGE = 5; // 何投票されたら接地点とみなすか
    bool FACE_PLY = false; // 出力をメッシュファイルにするかどうか

    std::string _project_name;
    std::string _data_path;
    std::string _projects_path;
    std::string _sfm_projects_path;

    int loadCameraParam(std::string file_name);
    int loadImages(std::string file_name);
    int loadMultipleCameraRts(std::string file_name);
    int loadOpenPoseData(std::string file_name);
    int trackTargetPerson();
    int findFootPrint();
    int paintFootPrint();
    int savePointClouds();
    int clickLegPoint();
    int paintClickedPoint();
    int findClickedFootPrint();
    void checkCameraParam();
    void savePlytoTxt(int i);
    void printVoteRecord();
    void votedFrameInit();
    void calculateSteppedFrame(int i);


    cv::Point3f imagePointTo3dPoint(cv::Point2f point);
    cv::Point2f worldPointToImagePoint(cv::Point3f point, Camera* camera);
    std::vector<int> findFootPrintAreaIn3D(cv::Point2f point, Camera camera);
    std::set<int> findFootPrintAreaIn3DSet(cv::Point2f point, Camera* camera, cv::Mat image);

    Model model;
    int imageWidth = 1920;
    int imageHeight = 1080;

    std::vector<int> right_vote;
    std::vector<int> left_vote;
    std::vector<std::pair<cv::Point2f, int>> clickLegPoints;

    cv::Scalar color;
    std::vector<std::pair<int, float>> RstepFrame;
    std::vector<std::pair<int, float>> LstepFrame;
    std::vector<Vote> RvoteRecord;
    std::vector<Vote> LvoteRecord;
private:

    std::vector<ImageInfo> image_infos;
    std::set<int> rightFootPrintID;
    std::set<int> leftFootPrintID;
    std::vector<float> cameraParam;
};


#endif //SFMDR_FOOTPRINT_FOOTPRINT_H
