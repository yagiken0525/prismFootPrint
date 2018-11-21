#include <iostream>
#include "FootPrint.h"

using namespace std;
using namespace cv;

int main() {
    FootPrint footPrint("test");
    footPrint.DIST_RANGE = 300; // 画像投影点の内どれくらいの距離の点を接地点とみなすか
    footPrint.FRAME_RANGE = 20; // 近傍何フレームで投票数を合計するか
    footPrint.VOTE_RANGE = 5; // 何投票されたら接地点とみなすか
    footPrint.CAMERA_NUM = 3; // 接地カメラ個数
    footPrint.CAMERA_FIRST_ID = 10; // 接地カメラの最小ID
    footPrint.FINISH_FRAME = 30; //何フレームまで実行するか
    footPrint.SEARCHING_RECT = 50; //次のフレームで周囲何ピクセルまで探索するか
    footPrint.VIDEO_TYPE = ".MP4"; // 動画拡張子
    footPrint.ORIGINAL_IMAGE_WIDTH = 1920; // 入力動画の幅
    footPrint.ORIGINAL_IMAGE_HEIGHT = 1080; //　入力動画の高さ
    footPrint.IMAGE_WIDTH = 1920; //　入力動画の高さ
    footPrint.IMAGE_HEIGHT = 1080; //　入力動画の高さ
    footPrint.SELECT_TRACKER_BY_CLICKING = false; // tracking対象を手動で指定するか
    footPrint.SHOW_TRACKING_RESULT = false; // tracking結果を表示するか
    footPrint.SHOW_REPROJECT_RESULT = false; // 点群の再投影結果を表示するか
    footPrint.CHECKER_BOARD_CALIBRATION = true; // 点群の再投影結果を表示するか
    footPrint.PLY_BLOCK_WIDTH = 10; // 点群の領域分割幅


//    // カメラパラメータ（3x3の３次元座標を２次元画像平面へ投影するための行列）
//    double k_elms[] = { 9.803769e+02, 0.000000e+00, 6.900000e+02, 0.000000e+00, 9.757217e+02, 2.441228e+02, 0.000000e+00, 0.000000e+00, 1.000000e+00 };
//    Matx33d K(k_elms);
//
//    // カメラウィジェット作成（青）
//    viz::WCameraPosition wcamera(K, 1.0, viz::Color::blue());
//
//    // 画面にカメラ追加
//    myWindow.showWidget("Camera", wcamera);

//    // カメラの姿勢を設定
//    Mat T = Mat::eye(4, 4, CV_32FC1);
//    T.at<float>(2, 3) = -2.0;  // Z座標の設定
//    myWindow.setWidgetPose("Camera", cv::Affine3f(T));

    //カメラパラメータの初期化と読み込み
    footPrint.cameraInfoInit();
    footPrint.loadAllCameraParam();
    footPrint.setImageResizehomography();

    //動画から画像への変換
//    footPrint.videoToImage();

    //OpenPoseによる検出
//    footPrint.detectHumanPose();

    //カメラ位置姿勢推定
    footPrint.estimateCameraPose();

    //床平面のplyファイル生成
    footPrint.generatePlaneModel();

    //plyファイルの読み込み
    footPrint.model.readModel(footPrint._projects_path + "planePoints.ply");

    //足あとの検出
    footPrint.estimateStepPositions();

    return 0;
}




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