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

#define CHANNEL 15

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

    typedef std::vector<bool> votelist;
    class VoteOfCamera{
    public:
        votelist _LvoteList;
        votelist _RvoteList;
        votelist _Lstepped;
        votelist _Rstepped;
    };
    typedef std::vector<VoteOfCamera> VoteOfPoint;

    class myImageInfo : public ImageInfo{
    public:
        std::vector<Vote> voteList;
        cv::Mat resultSizeIm;
    };

    class CameraInfo {
    public:
        int camID;
        std::vector<ImageInfo> imageList;
        Camera camera;
        std::vector<cv::Point2f> projPoints;
        cv::Mat2f projPointsMat;
    };

    class ModelInfo : public Model {
    public:
        cv::Point3f *modelBlocks;
        std::vector<VoteOfPoint> VoteOfPointsList;
    };

    class StepInfo{
    public:
        int frame;
        float stride;
        float speed;
        cv::Point2f stepPosition;
    };
    std::vector<StepInfo> stepInfoList;

    std::vector<FootPrint::CameraInfo> CameraInfoList;

    //しきい値
    OpenPosePerson prevTargetPerson;
    float DIST_RANGE; // 画像投影点の内どれくらいの距離の点を接地点とみなすか
    int VOTE_RANGE; // 何投票されたら接地点とみなすか
    int MIN_STRIDE;
    int VISUALIZE_FRAMES;
    int STEP_THRESHOLD; // 何投票されたら接地点とみなすか
    int CAMERA_NUM; // 接地カメラの個数
    int CAMERA_FIRST_ID; // 接地カメラの個数
    int IMAGE_NUM; // 接地カメラの個数
    int SEARCHING_RECT; // 接地カメラの個数
    std::string VIDEO_TYPE = "MP4"; // 接地カメラの個数
    int VIDEO_FPS = 30; // 動画のfps
    bool FACE_PLY = false; // 出力をメッシュファイルにするかどうか
    int FINISH_FRAME = 5; // 出力をメッシュファイルにするかどうか
    float ORIGINAL_IMAGE_WIDTH = 1920; // 出力をメッシュファイルにするかどうか
    float ORIGINAL_IMAGE_HEIGHT = 1080; // 出力をメッシュファイルにするかどうか
    float IMAGE_WIDTH = 1920; // 出力をメッシュファイルにするかどうか
    float IMAGE_HEIGHT = 1080; // 出力をメッシュファイルにするかどうか
    bool SELECT_TRACKER_BY_CLICKING = false;
    bool SHOW_TRACKING_RESULT = false;
    bool SHOW_REPROJECT_RESULT;
    bool CHECKER_BOARD_CALIBRATION;
    int FLOOR_WIDTH;
    int PLANE_WIDTH = 200;
    int POINT_DIST = 10;
    bool SHOW_TRAJECTORY;
    bool PLOT_ON_WARPED_IMAGE;
    bool USE_WEBCAM;
    bool ESTIMATE_RT;

    std::vector<ImageInfo> imWebCamList;
    Camera webCam;


    std::string _project_name;
    std::string _data_path;
    std::string _projects_path;
    std::string _video_path;
    std::string _openPose_path;
    std::string _camera_path;
    std::string _sfm_projects_path;

    void estimateStepWithMultipleCameras();
    void InitStepMaps();
    void estimateCameraPoseWithImage(Camera& cm);
    void renewResultInfoIm(cv::Mat im);
    void showResult();
    void InitVoteList();
    void InitVoteListWebCam(OpenPosePerson& ps, Camera cm);
    void vote(Camera* cm, cv::Point2f pt, const int imID, const int bdID);
    void votingToMap(const int x, const int y, const int imID, const int bdID);
    void estimateStepPositions();
    void voting();
    void countVotes();
    void loadAllImages();
    void cameraInfoInit();
    void videoToImage();
    void detectHumanPose();
    void loadAllCameraParam();
    void loadIntrinsicCameraParam();
    void loadExtrinsicCameraParam();
    void loadProjectionMatrix();
    void estimateCameraPoseWithCheckerBoard();
    void estimateCameraPoseWithClickingPoints();
    void findFootPrintFromCameraInfo();
    void generatePlaneModel();
    void estimateCameraPose();
    void projectPoints(CameraInfo& cm);
    void projectPointsForWebCam(Camera &cm);
    void separateBlocks();
    void setImageResizehomography();
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
    void estimateStepWithWebCam();
    void loadWebCamPram(Camera& cm);
    void estimateGroundPlane(cv::Mat points);
//    void estimateStepPositions();
    void outputTargetPersonInfo(CameraInfo &cam);
    cv::Mat3f generatePointCloudsAsMatrix(const int width, const int dist);

    cv::Point2f prevStep;
    cv::Point2f prevProjectedPt;
    cv::Point2f prevCoM;
    cv::Point2f prevPrevStep;
    int prevStepFrame;
    int prevPrevStepFrame;
    float walkingDistance;
    int numOfSteps;
    std::vector<std::vector<cv::Point2f>> stepedPointList;
    std::vector<cv::Mat> voteMapList;
    cv::Mat trajectoryMap;
    cv::Mat stepMap;
    cv::Mat originalStepMap;
    cv::Mat HeatMap;
    cv::Mat HeatVoteMap;
    cv::Mat ResultInfo;
    std::vector<std::vector<cv::Point2f>> detectedCornerList;
    cv::Point3f imagePointTo3dPoint(cv::Point2f point);
    cv::Point2f worldPointToImagePoint(cv::Point3f point, Camera* camera);
    std::vector<int> findFootPrintAreaIn3D(cv::Point2f point, Camera camera);
    std::set<int> findFootPrintAreaIn3DSet(cv::Point2f point, Camera* camera, cv::Mat image);
    cv::Mat imageResizeH;
    ModelInfo model;

    int imageWidth = 1920;
    int imageHeight = 1080;

    std::vector<int> right_vote;
    std::vector<int> left_vote;
    std::vector<std::pair<cv::Point2f, int>> clickLegPoints;
    std::vector<cv::Point3f> objectCorners;

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
