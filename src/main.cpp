#include <iostream>
#include "FootPrint.h"

using namespace std;

int main() {
    FootPrint footPrint("test");
    footPrint.DIST_RANGE = 10; // 画像投影点の内どれくらいの距離の点を接地点とみなすか
    footPrint.VOTE_RANGE = 8; // 何投票されたら接地点とみなすか
    footPrint.CAMERA_NUM = 3; // 接地カメラ個数
    footPrint.CAMERA_FIRST_ID = 10; // 接地カメラID
    footPrint.VIDEO_TYPE = ".MP4"; // 動画拡張子

    footPrint.FACE_PLY = true; // 出力をメッシュファイルにするかどうか

    //動画から画像への変換
    footPrint.videoToImage();

    //OpenPoseによる検出
    footPrint.detectHumanPose();

    //カメラ位置姿勢推定
    footPrint.loadIntrinsicCameraParam();
    footPrint.estimateCameraPose();

    //３次元復元
    footPrint.reconstruct3Dpose();

    //足あとの検出
    footPrint.loadAllCameraParam();
    footPrint.estimateStepPositions();



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