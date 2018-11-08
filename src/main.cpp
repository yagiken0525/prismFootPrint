#include <iostream>
#include "FootPrint.h"

using namespace std;

int main() {
    FootPrint footPrint("test");
    footPrint.DIST_RANGE = 50; // 画像投影点の内どれくらいの距離の点を接地点とみなすか
    footPrint.VOTE_RANGE = 8; // 何投票されたら接地点とみなすか
    footPrint.FRAME_RANGE = 15; // 連続何フレームで検索するか
    footPrint.CAMERA_NUM = 3; // 接地カメラ個数
    footPrint.CAMERA_FIRST_ID = 10; // 接地カメラID
    footPrint.LOAD_LIMIT = 5; // 接地カメラID
    footPrint.VIDEO_TYPE = ".MP4"; // 動画拡張子

    //カメラパラメータの初期化
    footPrint.cameraInfoInit();
    footPrint.loadAllCameraParam();


    //動画から画像への変換
//    footPrint.videoToImage();

    //OpenPoseによる検出
//    footPrint.detectHumanPose();

    //カメラ位置姿勢推定
    //footPrint.estimateCameraPose();

    //床平面のplyファイル作成
//    footPrint.generatePlaneModel();

    //足あとの検出
    footPrint.estimateStepPositions();



    //３次元復元
//    footPrint.loadAllImages();
//    footPrint.reconstruct3Dpose();
//    }else if(command == "ESTIMATE_STEP_POINTS"){
//    } if(1){
//        footPrint.model.readModel(footPrint._sfm_projects_path + "scene_mesh.ply");
//        footPrint.right_vote.resize(footPrint.model.vertices_num);
//        footPrint.left_vote.resize(footPrint.model.vertices_num);
//        footPrint.loadImages(footPrint._sfm_projects_path + "imagelist.txt");
//        footPrint.loadCameraParam(footPrint._data_path + "camera_param.txt");
//        footPrint.loadMultipleCameraRts(footPrint._sfm_projects_path + "camerapose.txt");
//        footPrint.loadOpenPoseData(footPrint._sfm_projects_path + "human_pose_info.txt");
//        footPrint.votedFrameInit();
//
//        footPrint.trackTargetPerson();
//        footPrint.findFootPrint();
//        footPrint.paintFootPrint();
//        footPrint.printVoteRecord();
//
//    }else if(command == "EXTRACT_STEP_AREA") {
//
//        //色ついた部分の抽出
//        footPrint.color = cv::Scalar(0, 0, 255);
//        footPrint.model.readModelWithColor(footPrint._projects_path + "result/result_mesh.ply", footPrint.color);
//        footPrint.savePointClouds();
//        footPrint.calculateSteppedFrame(0);
//        footPrint.savePlytoTxt(0);
//        footPrint.model.vertices.clear();
//
//        footPrint.color = cv::Scalar(0, 255, 0);
//        footPrint.model.readModelWithColor(footPrint._projects_path + "result/result_mesh.ply", footPrint.color);
//        footPrint.savePointClouds();
//        footPrint.calculateSteppedFrame(1);
//        footPrint.savePlytoTxt(1);
//    }
    return 0;
}