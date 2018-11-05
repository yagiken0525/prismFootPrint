#include <iostream>
#include "pointCloud/Model.h"
#include "FootPrint.h"

using namespace std;

int main() {
    FootPrint footPrint("IMG_1765");
    footPrint.DIST_RANGE = 10; // 画像投影点の内どれくらいの距離の点を接地点とみなすか
    footPrint.VOTE_RANGE = 8; // 何投票されたら接地点とみなすか
    footPrint.FACE_PLY = true; // 出力をメッシュファイルにするかどうか

    string command;
    command = "STEP_DITTECTION";
    command = "EXTRACT_STEP_AREA";
//
    if(command == "STEP_DITTECTION") {

        footPrint.model.readModel(footPrint._sfm_projects_path + "scene_mesh.ply");
        footPrint.right_vote.resize(footPrint.model.vertices_num);
        footPrint.left_vote.resize(footPrint.model.vertices_num);
        footPrint.loadImages(footPrint._sfm_projects_path + "imagelist.txt");
        footPrint.loadCameraParam(footPrint._data_path + "camera_param.txt");
        footPrint.loadMultipleCameraRts(footPrint._sfm_projects_path + "camerapose.txt");
        footPrint.loadOpenPoseData(footPrint._sfm_projects_path + "human_pose_info.txt");
        footPrint.votedFrameInit();

        footPrint.trackTargetPerson();
        footPrint.findFootPrint();
        footPrint.paintFootPrint();
        footPrint.printVoteRecord();

    }else if(command == "EXTRACT_STEP_AREA") {

        //色ついた部分の抽出
        footPrint.color = cv::Scalar(0, 0, 255);
        footPrint.model.readModelWithColor(footPrint._projects_path + "result/result_mesh.ply", footPrint.color);
        footPrint.savePointClouds();
        footPrint.calculateSteppedFrame(0);
        footPrint.savePlytoTxt(0);
        footPrint.model.vertices.clear();

        footPrint.color = cv::Scalar(0, 255, 0);
        footPrint.model.readModelWithColor(footPrint._projects_path + "result/result_mesh.ply", footPrint.color);
        footPrint.savePointClouds();
        footPrint.calculateSteppedFrame(1);
        footPrint.savePlytoTxt(1);

    }
    return 0;
}