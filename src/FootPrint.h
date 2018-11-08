//
// Created by yagi on 18/07/24.
//

#ifndef SFMDR_FOOTPRINT_FOOTPRINT_H
#define SFMDR_FOOTPRINT_FOOTPRINT_H

#include "pointCloud/Model.h"
#include "image/ImageInfo.h"
#include "camera/Camera.h"
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
        _video_path = _data_path + "Projects/" + _project_name + "/videos/";
        _openPose_path = _data_path + "Projects/" + _project_name + "/openPoseData/";
        _camera_path = _data_path + "/Camera/";
        _sfm_projects_path = "/home/yagi/sfmDR/projects/" + _project_name + "/";
    };
    ~FootPrint(){};

    class CameraInfo {
    public:
        std::vector<ImageInfo> imageList;
        Camera camera;
        std::vector<cv::Point2d> projPoints;
    };

    class ModelInfo : public Model {
    public:
        std::vector<std::vector<std::vector<bool>>> votelist;
        std::vector<std::vector<bool>> ifSteped;
    };

    std::vector<FootPrint::CameraInfo> CameraInfoList;

    //しきい値
    float DIST_RANGE = 30; // 画像投影点の内どれくらいの距離の点を接地点とみなすか
    int VOTE_RANGE = 5; // 何投票されたら接地点とみなすか
    int FRAME_RANGE = 5; // 何投票されたら接地点とみなすか
    int CAMERA_NUM = 3; // 接地カメラの個数
    int CAMERA_FIRST_ID = 3; // 接地カメラの個数
    std::string VIDEO_TYPE = "MP4"; // 接地カメラの個数
    bool FACE_PLY = false; // 出力をメッシュファイルにするかどうか
    int LOAD_LIMIT = 5; // 出力をメッシュファイルにするかどうか
    int IMAGE_WIDTH = 1920; // 出力をメッシュファイルにするかどうか
    int IMAGE_HEIGHT = 1080; // 出力をメッシュファイルにするかどうか

    std::string _project_name;
    std::string _data_path;
    std::string _projects_path;
    std::string _video_path;
    std::string _openPose_path;
    std::string _camera_path;
    std::string _sfm_projects_path;

    void loadAllImages();
    void cameraInfoInit();
    void videoToImage();
    void detectHumanPose();
    void loadAllCameraParam();
    void loadIntrinsicCameraParam();
    void loadExtrinsicCameraParam();
    void loadProjectionMatrix();
    void estimateCameraPose();
    void findFootPrintFromCameraInfo(CameraInfo& cam);
    void generatePlaneModel();

    int loadImages(std::string file_name, std::vector<ImageInfo>& imageInfoList, int lim = 1000);
    int loadMultipleCameraRts(std::string file_name, std::vector<ImageInfo>& imageInfoList);
    int loadOpenPoseData(std::string file_name, std::vector<ImageInfo>& imageInfoList, int lim = 1000);
    int trackTargetPerson(std::vector<ImageInfo>& imageInfoList);
    int findFootPrint(std::vector<ImageInfo>& imageInfoList, Camera cam);
    int paintFootPrint();
    int savePointClouds();
    int clickLegPoint(std::vector<ImageInfo>& imageInfoList);
    int paintClickedPoint();
    int findClickedFootPrint(std::vector<ImageInfo>& imageInfoList);
    void checkCameraParam();
    void savePlytoTxt(int i);
    void printVoteRecord();
    void votedFrameInit();
    void calculateSteppedFrame(int i);
    void reconstruct3Dpose();
    void estimateGroundPlane(cv::Mat points);
    void estimateStepPositions();


    cv::Point3f imagePointTo3dPoint(cv::Point2f point);
    cv::Point2f worldPointToImagePoint(cv::Point3f point, Camera* camera);
    std::vector<int> findFootPrintAreaIn3D(cv::Point2f point, Camera camera);
    std::set<int> findFootPrintAreaIn3DSet(cv::Point2f point, Camera* camera, cv::Mat image);

    ModelInfo model;
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

//    std::vector<std::vector<ImageInfo>> image_lists_list;
    std::set<int> rightFootPrintID;
    std::set<int> leftFootPrintID;
//    std::vector<float> cameraParam;
};


#endif //SFMDR_FOOTPRINT_FOOTPRINT_H
