#include <iostream>
#include "FootPrint.h"

using namespace std;
using namespace cv;


int main() {
    FootPrint footPrint("test");
    footPrint.DIST_RANGE = 300; // 画像投影点の内どれくらいの距離の点を接地点とみなすか
    footPrint.STEP_THRESHOLD = 10; // 接地判定のための投票数しきい値
    footPrint.VOTE_RANGE = 5; // 近接何ピクセルまで投票するか
    footPrint.MIN_STRIDE = 25; // 一歩とカウントする際の最低歩幅(cm)
    footPrint.VISUALIZE_FRAMES = 100; // 最近何フレーム分の接地点を表示するか
    footPrint.CAMERA_NUM = 1; // 接地カメラ個数
    footPrint.CAMERA_FIRST_ID = 12; // 接地カメラの最小ID
    footPrint.FINISH_FRAME = 600; //何フレームまで実行するか
    footPrint.SEARCHING_RECT = 50; //次のフレームで周囲何ピクセルまで探索するか
    footPrint.VIDEO_TYPE = ".MP4"; // 動画拡張子
    footPrint.VIDEO_FPS = 30; // 動画拡張子
    footPrint.ORIGINAL_IMAGE_WIDTH = 1920; // 入力動画の幅
    footPrint.ORIGINAL_IMAGE_HEIGHT = 1080; //　入力動画の高さ
    footPrint.IMAGE_WIDTH = 1920; //　入力動画の高さ
    footPrint.IMAGE_HEIGHT = 1080; //　入力動画の高さ
    footPrint.SELECT_TRACKER_BY_CLICKING = false; // tracking対象を手動で指定するか
    footPrint.SHOW_TRACKING_RESULT = false; // tracking結果を表示するか
    footPrint.SHOW_REPROJECT_RESULT = false; // 点群の再投影結果を表示するか
    footPrint.CHECKER_BOARD_CALIBRATION = true; // キャリブレーション行うか
    footPrint.PLANE_WIDTH = 200; // 点群の領域分割幅
    footPrint.POINT_DIST = 10; // 点群の領域分割幅
    footPrint.SHOW_TRAJECTORY = true; // trajectory表示
    footPrint.PLOT_ON_WARPED_IMAGE = true; // 床画像に足あと投影

//    footPrint.voteMap = cv::Mat::zeros(footPrint.PLANE_WIDTH * 2, footPrint.PLANE_WIDTH * 2, CV_8UC(CHANNEL));
    footPrint.trajectoryMap = cv::Mat::ones(footPrint.PLANE_WIDTH * 2, footPrint.PLANE_WIDTH * 2, CV_8UC3);
    footPrint.stepMap = cv::Mat::ones(footPrint.PLANE_WIDTH * 2, footPrint.PLANE_WIDTH * 2, CV_8UC3);
    footPrint.HeatMap = cv::Mat::ones(footPrint.PLANE_WIDTH * 2, footPrint.PLANE_WIDTH * 2, CV_8UC3);
    footPrint.HeatVoteMap = cv::Mat::zeros(footPrint.PLANE_WIDTH * 2, footPrint.PLANE_WIDTH * 2, CV_32F);
    footPrint.ResultInfo = cv::Mat::ones(footPrint.PLANE_WIDTH * 2, footPrint.PLANE_WIDTH * 2, CV_8UC3);

//    footPrint.trajectoryMap * 255;
    //カメラパラメータの初期化と読み込み
    footPrint.cameraInfoInit();
    footPrint.loadAllCameraParam();

    //動画から画像への変換
//    footPrint.videoToImage();

    //OpenPoseによる検出
//    footPrint.detectHumanPose();

    //カメラ位置姿勢推定
//    footPrint.estimateCameraPose();

    //床平面の点群生成
    cv::Mat3f plane = footPrint.generatePointCloudsAsMatrix(footPrint.PLANE_WIDTH, footPrint.POINT_DIST);

    for(int camID = 0; camID < footPrint.CAMERA_NUM; camID++) {
        string CAM_NAME = "cam" + to_string(camID + footPrint.CAMERA_FIRST_ID);
        FootPrint::CameraInfo* cm = &footPrint.CameraInfoList[camID];
        footPrint.loadImages(footPrint._projects_path + "openPoseData/" + CAM_NAME + "/imagelist.txt", cm->imageList, footPrint.FINISH_FRAME);
        footPrint.loadOpenPoseData(footPrint._projects_path + "openPoseData/" + CAM_NAME + "/human_pose_info.txt",
                         cm->imageList, footPrint.FINISH_FRAME);
        footPrint.trackTargetPerson(cm->imageList);
        footPrint.projectPoints(*cm);
    }

    footPrint.estimateStepPositions();


//    footPrint.voting();
//    countVotes();
//    footPrint.paintFootPrint();




    return 0;
}




//３次元復元
//    footPrint.loadAllImages();
//    footPrint.reconstruct3Dpose();
//    }else if(command == "ESTIMATE_STEP_POINTS"){
//    } if(1){
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


//    cv::VideoCapture cap(0);//デバイスのオープン
//    //cap.open(0);//こっちでも良い．
//
//    if(!cap.isOpened())//カメラデバイスが正常にオープンしたか確認．
//    {
//        //読み込みに失敗したときの処理
//        return -1;
//    }
//
//    while(1)//無限ループ
//    {
//        cv::Mat frame;
//        cap >> frame; // get a new frame from camera
//
//        //
//        //取得したフレーム画像に対して，クレースケール変換や2値化などの処理を書き込む．
//        //
//
//        cv::imshow("window", frame);//画像を表示．
//
//        int key = cv::waitKey(1);
//        if(key == 113)//qボタンが押されたとき
//        {
//            break;//whileループから抜ける．
//        }
//        else if(key == 115)//sが押されたとき
//        {
//            //フレーム画像を保存する．
//            cv::imwrite("img.png", frame);
//        }
//    }
//    cv::destroyAllWindows();
//    return 0;

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

//    footPrint.generatePlaneModel();
//    for(int i = 0; i < plane.cols; i++){
//        for(int j = 0; j < plane.rows; j++){
//            cout << plane(cv::Point(j,i)) << endl;
//        }
//    }

//plyファイルの読み込み
//    footPrint.model.readModel(footPrint._projects_path + "planePoints.ply");

//plyファイルをブロックに分割