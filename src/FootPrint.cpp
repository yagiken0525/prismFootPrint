//
// Created by yagi on 18/07/24.
//

#include "FootPrint.h"
#include "videoToImage/videoToImage.h"
#include "pointCloud/Model.h"
#include "basicFunction/basicFunction.h"
#include "openPose/myOpenPose.h"

using namespace std;

//Loading Camera Parameters
void FootPrint::loadAllCameraParam() {
    loadIntrinsicCameraParam();
    loadExtrinsicCameraParam();

    for(int i=0; i < this->CAMERA_NUM; i++) {
        cout << this->CameraInfoList[i].camera._A << endl;
        cout << this->CameraInfoList[i].camera._Rvec << endl;
        cout << this->CameraInfoList[i].camera._Tvec << endl;
        cout << this->CameraInfoList[i].camera._dist << endl;
    }

//    loadProjectionMatrix();
}

void FootPrint::cameraInfoInit(){
    for (int i = 0; i < this->CAMERA_NUM; i++){
        CameraInfo camIn;
        Camera cam;
        camIn.camID = i;
        camIn.camera = cam;
        this->CameraInfoList.push_back(camIn);
    }
}

void FootPrint::loadIntrinsicCameraParam() {
    for (int camID = this->CAMERA_FIRST_ID; camID < this->CAMERA_FIRST_ID + this->CAMERA_NUM; camID++) {
        string cameraName = "gopro" + to_string(camID);
        this->CameraInfoList[camID - this->CAMERA_FIRST_ID].camera.loadCameraA(this->_camera_path + cameraName + "/" + cameraName + ".txt");
    }
}

void FootPrint::loadExtrinsicCameraParam() {
    for (int camID = this->CAMERA_FIRST_ID; camID < this->CAMERA_FIRST_ID + this->CAMERA_NUM; camID++) {
        string cameraName = "gopro" + to_string(camID);
        this->CameraInfoList[camID - this->CAMERA_FIRST_ID].camera.loadCameraRt(this->_camera_path + cameraName + "/cameraRt.txt");
    }
}

void FootPrint::loadProjectionMatrix() {
    for (int camID = this->CAMERA_FIRST_ID; camID < this->CAMERA_FIRST_ID + this->CAMERA_NUM; camID++) {
        string cameraName = "gopro" + to_string(camID);
        this->CameraInfoList[camID - this->CAMERA_FIRST_ID].camera.loadCameraP(this->_camera_path + cameraName + "/" + cameraName + ".txt");
    }
}


//Load Images
int FootPrint::loadImages(string file_path, vector<ImageInfo>& imageInfoList, int LIMIT) {

    cout << "[Loading image]:" << endl;

    //画像リストopen
    ifstream ifs(file_path);

    //imshowうまくいかない時用
    string line;
    int string_size = 0;
    int image_num = 0;

    // cv::namedWindow("image", CV_WINDOW_NORMAL);
    while (getline(ifs, line)) {

        //imshowがうまくいかないときここ原因(下4行をコメントアウト)
//        if (string_size == 0 || (string_size + 1) == line.size()) {
//            line.erase(line.size() - 1);
//        }
//        string_size = line.size();

        //ImageInfoに画像を格納していく
        ImageInfo image_info;

        //カラー、グレースケール,hsv
        cv::Mat image = cv::imread(line);

        //画像格納
        image_info.image = image;

        imageInfoList.push_back(image_info);

        image_num++;

        if(image_num == LIMIT)
            break;
    }
}


int FootPrint::loadOpenPoseData(string file_name, vector<ImageInfo>& imageInfoList, int LIMIT){

    cout << "[Loading open pose data]:" << endl;
    //OpenPoseのデータ読みこみ
    ifstream ifs(file_name);
    if(!ifs.is_open()){
        std::cout << "cannot open "<< file_name << std::endl;
        return false;
    }
    string line;
    OpenPosePerson person;
    vector<OpenPosePerson> persons;
    vector<vector<OpenPosePerson>> allFramePersons;

    bool first_person = true;
    int frame_counter = 0;
    int frame_num = 0;
    vector<int> person_detected_frameID;

    while (getline(ifs, line)) {

        vector<string> coords = yagi::split(line, ' ');

        if (coords.size() == 5) {
            if (!first_person) {
                OpenPosePerson dummy_person = person;
                persons.push_back(dummy_person);
                person.clearBodyCoord();

                if (coords[1] == "0") {
                    vector<OpenPosePerson> dummy_persons = persons;
                    allFramePersons.push_back(dummy_persons);
                    person_detected_frameID.push_back(frame_counter);
                    persons.clear();
                    frame_counter++;
                }
            }
        } else {
            person.setBodyCoord(coords);
            first_person = false;
        }

        if(frame_counter == LIMIT)
            break;
    }
    cout << allFramePersons.size() << endl;

    for (int i = 0; i < person_detected_frameID.size(); i++) {
        imageInfoList[person_detected_frameID[i]].persons = allFramePersons[i];
    }
}


//グローバル変数
struct mouseParam {
    int x;
    int y;
};
bool flag = false;
cv::Point2f tracking_runner_point;


//コールバック関数
void runnerClickCallBackFunc(int eventType, int x, int y, int flags, void *userdata) {
    switch (eventType) {
        case cv::EVENT_LBUTTONUP:
            tracking_runner_point.x = x;
            tracking_runner_point.y = y;
            flag = true;
    }
}


int FootPrint::trackTargetPerson(vector<ImageInfo>& imageInfoList){
    vector<cv::Scalar> colors;
    yagi::setColor(&colors);

    cout << "[Track target person]:" << endl;

    //最初のフレームで対象人物クリック
    mouseParam mouseEvent;
    string window_name = "click target person";
    cv::namedWindow(window_name, CV_WINDOW_AUTOSIZE);
    cv::setMouseCallback(window_name, runnerClickCallBackFunc, &mouseEvent);
    bool target_selected = false;
    int clicked_frame = 0;


    for(ImageInfo im: imageInfoList) {
        cv::Mat image = im.image.clone();
        while (im.persons.size() > 0) {
            cv::imshow(window_name, image);
            int key = cv::waitKey(1);

            if (flag) {
                //click point格納
                flag = false;
                cv::circle(image, tracking_runner_point, 2, colors[0], 2);
                cout << "clicked_point = " << tracking_runner_point << endl;
            }

            if(key == 'n'){
                break;
            }

            if (key == 'q') {
                target_selected = true;
                break;
            }
        }
        if (target_selected)
            break;
        clicked_frame++;
    }

    int frameID = 0;
    cv::Point2f prePt = tracking_runner_point;
    OpenPosePerson preHb;

    bool target_found = false;
    for (ImageInfo im : imageInfoList) {
        if (frameID >= clicked_frame) {
            float minDist = 500;
            int minId = 0;
            int hbID = 0;
            cv::Point2f minPt;
            OpenPosePerson minHb;
            cv::Mat dum = im.image.clone();

            for (OpenPosePerson hb: im.persons) {
                vector<cv::Point2f> bodyCoords = hb.getBodyCoord();
                if (target_found == false) {
                    for (int i = 0; i < bodyCoords.size(); i++) {
                        if ((i == 0) || (i == 14) || (i == 15) || (i == 16) || (i == 17)) {
                            float dist = yagi::calc2PointDistance(bodyCoords[i], prePt);
                            if (minDist > dist) {
                                minDist = dist;
                                minId = hbID;
                                minPt = bodyCoords[i];
                                minHb = hb;
                            }
                        }
                    }
                } else {
                    float sum_dist = 0;
                    int parts_id = 0;
                    int parts_num = 0;

                    for (int i = 0; i < bodyCoords.size(); i++) {
                        for (cv::Point2f pre_pt: preHb.getBodyCoord()) {
                            if ((parts_id != 4) && (parts_id != 7) && (parts_id != 10) && (parts_id != 13)) {
                                if ((bodyCoords[i].x != 0) && (pre_pt.x != 0)) {
                                    parts_num++;
                                    sum_dist += yagi::calc2PointDistance(bodyCoords[i], pre_pt);
                                }
                            }
                        }
                    }
                    sum_dist /= parts_num;

                    if (sum_dist < minDist) {
                        minDist = sum_dist;
                        minId = hbID;
                        minPt = im.persons[minId].getBodyCoord()[0];
                        minHb = hb;
                    }
                }
                hbID++;
            }
            if (minDist != 500.0) {
                imageInfoList[frameID].persons[minId].humanID = 1;
                prePt = minPt;
                preHb = minHb;
                target_found = true;
                for (cv::Point2f pt : im.persons[minId].getBodyCoord()) {
                    cv::circle(im.image, pt, 2, colors[3], 2);
                }
//                cv::imshow("target_person", im.image);
//                cv::waitKey(0);
            }

//            cv::imshow("targetRunner", im.image);
//            cv::waitKey(10);

            if (frameID == (imageInfoList.size() - 1))
                break;
        }
        frameID++;
    }
    cv::destroyAllWindows();
}

int FootPrint::clickLegPoint(vector<ImageInfo>& imageInfoList){
    vector<cv::Scalar> colors;
    yagi::setColor(&colors);

    cout << "[Track target person]:" << endl;

    //最初のフレームで対象人物クリック
    mouseParam mouseEvent;
    string window_name = "click target person";
    cv::namedWindow(window_name, CV_WINDOW_AUTOSIZE);
    cv::setMouseCallback(window_name, runnerClickCallBackFunc, &mouseEvent);
    bool target_selected = false;
    int clicked_frame = 0;


    for(ImageInfo im: imageInfoList) {
        cv::Mat image = im.image.clone();
        while (im.persons.size() > 0) {
            cv::imshow(window_name, image);
            int key = cv::waitKey(1);

            if (flag) {
                //click point格納
                flag = false;
                cv::circle(image, tracking_runner_point, 2, colors[0], 2);
                pair<cv::Point2f, int> clickedPoint(tracking_runner_point, clicked_frame);
                this->clickLegPoints.push_back(clickedPoint);
                cout << "clicked_point = " << tracking_runner_point << endl;
            }

            if(key == 'n'){
                break;
            }

            if (key == 'q') {
                target_selected = true;
                break;
            }
        }
        if (target_selected)
            break;
        clicked_frame++;
    }

    for(pair<cv::Point2f, int> pt : this->clickLegPoints){
        cout << "Frame: " << pt.second << " Pt: " << pt.first << endl;
        cout << "Legpoint" << imageInfoList[pt.second].persons[0].getBodyCoord()[10] << endl;
        cout << "Legpoint" << imageInfoList[pt.second].persons[0].getBodyCoord()[13] << endl;
    }

//    int frameID = 0;
//    cv::Point2f prePt = tracking_runner_point;
//    OpenPosePerson preHb;
//
//    for (ImageInfo im : imageInfoList) {
//        if (frameID >= clicked_frame) {
//            float minDist = 100;
//            int minId = 0;
//            int hbID = 0;
//            bool target_found = false;
//            cv::Point2f minPt;
//            OpenPosePerson minHb;
//            cv::Mat dum = im.image.clone();
////            cout << im.persons.size() << endl;
//            for (OpenPosePerson hb: im.persons) {
//                vector<cv::Point2f> bodyCoords = hb.getBodyCoord();
//                if (target_found == false) {
//                    for (int i = 0; i < bodyCoords.size(); i++) {
//                        if ((i == 0) || (i == 14) || (i == 15) || (i == 16) || (i == 17)) {
//                            float dist = yagi::calc2PointDistance(bodyCoords[i], prePt);
////                            cout << "dist = " << dist << endl;
//                            if (minDist > dist) {
//                                minDist = dist;
//                                minId = hbID;
//                                minPt = bodyCoords[i];
//                                minHb = hb;
//                            }
//                        }
//                    }
//                } else {
//                    float sum_dist = 0;
//                    int parts_id = 0;
//                    int parts_num = 0;
//
//                    for (int i = 0; i < bodyCoords.size(); i++) {
//                        for (cv::Point2f pre_pt: preHb.getBodyCoord()) {
//                            if ((parts_id != 4) && (parts_id != 7) && (parts_id != 10) && (parts_id != 13)) {
//                                if ((bodyCoords[i].x != 0) && (pre_pt.x != 0)) {
//                                    parts_num++;
//                                    sum_dist += yagi::calc2PointDistance(bodyCoords[i], pre_pt);
//                                }
//                            }
//                        }
//                    }
//                    sum_dist /= parts_num;
//
//                    if (sum_dist < minDist) {
//                        minDist = sum_dist;
//                        minId = hbID;
//                        minPt = im.persons[minId].getBodyCoord()[0];
//                        minHb = hb;
//                    }
//                }
//                hbID++;
//
//
//                if (minDist != 100.0) {
//                    imageInfoList[frameID].persons[minId].humanID = 1;
//                    prePt = minPt;
//                    preHb = minHb;
//                    target_found = true;
//                    for (cv::Point2f pt : im.persons[minId].getBodyCoord()) {
//                        cv::circle(im.image, pt, 2, colors[3], 2);
//                    }
////                    cv::imshow("target_person", im.image);
////                    cv::waitKey(1);
//                }
//            }
//
////            cv::imshow("targetRunner", im.image);
////            cv::waitKey(10);
//
//            if (frameID == (imageInfoList.size() - 1))
//                break;
//        }
//        frameID++;
//    }
    cv::destroyAllWindows();
}

void FootPrint::voting() {

    int frameNum = this->CameraInfoList[0].imageList.size();

    for (CameraInfo cm : this->CameraInfoList) {
        for (cv::Point2f pt : cm.projPoints) {
            vector<bool> LlegVote;
            vector<bool> RlegVote;
            for (ImageInfo im : cm.imageList) {
                for (OpenPosePerson person: im.persons) {
                    if (person.humanID == 1) {
                        float dist;

                        //右足の投票
                        cv::Point2f RlegPt = person.getBodyCoord()[13];
                        RlegPt.x /= (640 / im.image.cols);
                        RlegPt.y /= (320 / im.image.rows);

                        //左足の投票
                        cv::Point2f LlegPt = person.getBodyCoord()[10];
                        LlegPt.x *= (640 / im.image.cols);
                        LlegPt.y *= (320 / im.image.rows);

                        dist = yagi::calc2PointDistance(pt, LlegPt);
                        if ((dist < this->DIST_RANGE) && (dist != 0.0)) {
                            LlegVote.push_back(true);
                        } else {
                            LlegVote.push_back(false);
                        }

                        dist = yagi::calc2PointDistance(pt, RlegPt);
                        if ((dist < this->DIST_RANGE) && (dist != 0.0)) {
                            RlegVote.push_back(true);
                        } else {
                            RlegVote.push_back(false);
                        }
                    }
                }
            }
            VoteList voteList;
            voteList.LlegVote = LlegVote;
            voteList.RlegVote = RlegVote;
            this->CameraInfoList[cm.camID].voteOfEachPointList.push_back(voteList);
        }
    }
}

void FootPrint::countVotes(){
    int IMAGE_NUM = this->CameraInfoList[0].imageList.size();
    for (CameraInfo cm : this->CameraInfoList) {
        for (int ptID = 0; ptID < cm.projPoints.size(); ptID++) {
            for (int legID = 0; legID < 2; legID++) {
                vector<bool> ifStepped;
                vector<bool> voteList;
                if (legID == 0) {
                    voteList = cm.voteOfEachPointList[ptID].LlegVote;
                    for (int frameID = 0; frameID < IMAGE_NUM; frameID++) {
                        if ((frameID > FRAME_RANGE / 2) && (frameID < IMAGE_NUM + (FRAME_RANGE / 2))) {
                            int votePt = 0;
                            for (int voteID = 0; voteID < VOTE_RANGE; voteID++) {
                                if (voteList[voteID])
                                    votePt++;
                            }
                            if (votePt > VOTE_RANGE)
                                ifStepped.push_back(true);
                            else
                                ifStepped.push_back(false);
                        }
                    }
                    this->CameraInfoList[cm.camID].voteOfEachPointList[ptID].LifStepped = ifStepped;
                }else {
                    voteList = cm.voteOfEachPointList[ptID].RlegVote;
                    for (int frameID = 0; frameID < IMAGE_NUM; frameID++) {
                        if ((frameID > FRAME_RANGE / 2) && (frameID < IMAGE_NUM + (FRAME_RANGE / 2))) {
                            int votePt = 0;
                            for (int voteID = 0; voteID < VOTE_RANGE; voteID++) {
                                if (voteList[voteID])
                                    votePt++;
                            }
                            if (votePt > VOTE_RANGE)
                                ifStepped.push_back(true);
                            else
                                ifStepped.push_back(false);
                        }
                    }
                    this->CameraInfoList[cm.camID].voteOfEachPointList[ptID].RifStepped = ifStepped;
                }
            }
        }
    }
}


int FootPrint::findFootPrint(vector<ImageInfo>& imageInfoList, Camera cam){
    int frame_num = 0;
    for(ImageInfo im: imageInfoList){
        if (frame_num % 10 == 0) {
            cout << frame_num << endl;
        }

        //固定カメラなのでコメントアウト
//        cv::Mat R_mat = cv::Mat::zeros(3,3,CV_32F);
//
//        for(int i = 0; i < 3; i++){
//            for(int j = 0; j < 3; j++){
//                R_mat.at<float>(j, i) = im.cam._R[i * 3 + j];
//            }
//        }
//
//        im.camera.R_inv = R_mat.inv();

        for(OpenPosePerson person: im.persons){
            if (person.humanID == 1){
                cv::Point2f leftLeg = person.getBodyCoord()[10];
                cv::Point2f rightLeg = person.getBodyCoord()[13];

                //足領域が3次元点群のどこにあたるか
                set<int> leftLegAreaID = this->findFootPrintAreaIn3DSet(leftLeg, &im.camera, im.image);
                set<int> rightLegAreaID = this->findFootPrintAreaIn3DSet(rightLeg, &im.camera, im.image);

                //vectorに投票
                if(frame_num > 0) {
                    for (int id: leftLegAreaID) {
                        this->left_vote[id] += 1;
//                        LvoteRecord[id].frameNum.push_back(frame_num);
                    }
                    for (int id: rightLegAreaID) {
                        this->right_vote[id] += 1;
//                        RvoteRecord[id].frameNum.push_back(frame_num);
                    }
                }
            }
        }
        frame_num++;
    }

    return 1;
}

int FootPrint::findClickedFootPrint(vector<ImageInfo>& imageInfoList){
    int frame_num = 0;
    for (ImageInfo im: imageInfoList) {
        for (pair<cv::Point2f, int> pt : this->clickLegPoints) {
            if(frame_num == pt.second) {
                for (int i = 0; i < this->model.vertices_num; i++) {
                    cv::Point2f projected_point = this->worldPointToImagePoint(this->model.vertices[i], &im.camera);
                    float dist = yagi::calc2PointDistance(pt.first, projected_point);
                    if (dist < this->DIST_RANGE && dist != 0.0) {
                        this->left_vote[i] += 1;
                    }
                }
            }
        }
        frame_num++;
    }
    return 1;
}

cv::Point3f FootPrint::imagePointTo3dPoint(cv::Point2f point){
//    cv::Point3f vector_to_world;
//    vector_to_world.x = (point.x - this->cameraParam[2]) / this->cameraParam[0];
//    vector_to_world.y = (point.y - this->cameraParam[5]) / this->cameraParam[4];
//    vector_to_world.z = 1.0;
//    return vector_to_world;
}

cv::Point2f FootPrint::worldPointToImagePoint(cv::Point3f point, Camera* camera){


////    for project points
////    世界座標系からカメラ座標系への変換
//    cv::Point3f camera_coord_point;
//    vector<float> R = camera->_R;
//    cv::Point3f T = camera->_T;
//
////    cv::Mat R_mat = cv::Mat::zeros(3,3,CV_64F);
////    for(int i = 0; i < 3; i++){
////        for(int j = 0; j < 3; j++){
////            R_mat.at<double>(j, i) = R[i * 3 + j];
////        }
////    }
//////    cout << R_mat << endl;
////
////    cv::Mat T_mat = cv::Mat::zeros(3, 1, CV_64F);
////    T_mat.at<double>(0, 0) = -T.x;
////    T_mat.at<double>(1, 0) = -T.y;
////    T_mat.at<double>(2, 0) = -T.z;
//////    cout << T_mat << endl;
////
////    cv::Mat A_mat = cv::Mat::zeros(3,3,CV_64F);
////    for(int i = 0; i < 3; i++){
////        for(int j = 0; j < 3; j++){
////            A_mat.at<double>(j, i) = cameraParam[i * 3 + j];
////        }
////    }
////
////    cv::Mat Coef = cv::Mat::zeros(4, 1, CV_64F);
////
////    vector<cv::Point3f> points;
////    vector<cv::Point2d> image_points;
////    points.push_back(point);
//////    cout <<point <<  points[0] << endl;
////
////    cv::Mat r_vec;
////    cv::Rodrigues(R_mat.inv(), r_vec);
////    cv::projectPoints(points, R_mat.inv(), T_mat, A_mat, Coef, image_points);
//
//
////    camera_coord_point.x = R.at<float>(0, 0)*point.x + R.at<float>(0, 1)*point.y + R.at<float>(0, 2)*point.z;
////    camera_coord_point.y = R.at<float>(1, 0)*point.x + R.at<float>(1, 1)*point.y + R.at<float>(1, 2)*point.z;
////    camera_coord_point.z = R.at<float>(2, 0)*point.x + R.at<float>(2, 1)*point.y + R.at<float>(2, 2)*point.z;
////    camera_coord_point.x = R[0]*point.x + R[3]*point.y + R[6]*point.z;
////    camera_coord_point.y = R[1]*point.x + R[4]*point.y + R[7]*point.z;
////    camera_coord_point.z = R[2]*point.x + R[5]*point.y + R[8]*point.z;
//
//
//    camera_coord_point = point;
//
////    camera_coord_point.x - T.x;
////    camera_coord_point.y - T.y;
////    camera_coord_point.z - T.z;
//    point.x -= T.x;
//    point.y -= T.y;
//    point.z -= T.z;
//    camera_coord_point.x = camera->R_inv.at<float>(0, 0)*point.x + camera->R_inv.at<float>(0, 1)*point.y + camera->R_inv.at<float>(0, 2)*point.z;
//    camera_coord_point.y = camera->R_inv.at<float>(1, 0)*point.x + camera->R_inv.at<float>(1, 1)*point.y + camera->R_inv.at<float>(1, 2)*point.z;
//    camera_coord_point.z = camera->R_inv.at<float>(2, 0)*point.x + camera->R_inv.at<float>(2, 1)*point.y + camera->R_inv.at<float>(2, 2)*point.z;
//
////    camera_coord_point.x = R[0]*point.x + R[1]*point.y + R[2]*point.z;
////    camera_coord_point.y = R[3]*point.x + R[4]*point.y + R[5]*point.z;
////    camera_coord_point.z = R[6]*point.x + R[7]*point.y + R[8]*point.z;
//
////    camera_coord_point.x - 8.91;
////    camera_coord_point.y - 0.33;
////    camera_coord_point.z + 4.49;
//
////    camera_coord_point = point;
//
//    //カメラ座標系から正規化画像座標系への変換
//    cv::Point2f normalized_point;
//    normalized_point.x = camera_coord_point.x / camera_coord_point.z;
//    normalized_point.y = camera_coord_point.y / camera_coord_point.z;
//
//    //正規化画像座標系から画像座標系への変換
//    cv::Point2f image_point;
////    cout << this->cameraParam[0] << " " << this->cameraParam[2] << endl;
////    cout << this->cameraParam[4] << " " << this->cameraParam[5] << endl;
////    image_point.x = (normalized_point.x * 800) + 960;
////    image_point.y = (normalized_point.y * 800) + 540;
//    image_point.x = (normalized_point.x * this->cameraParam[0]) + this->cameraParam[2];
//    image_point.y = (normalized_point.y * this->cameraParam[4]) + this->cameraParam[5];
//
////    cout << image_point << endl;
////    cout << image_points[0] << endl;
//
//    return image_point;
////    return image_points[0];
}

vector<int> FootPrint::findFootPrintAreaIn3D(cv::Point2f point, Camera camera){
    vector<int> foot_area_vertices_id;
    for (int i = 0; i < this->model.vertices_num; i++) {
        cv::Point2f projected_point = this->worldPointToImagePoint(this->model.vertices[0], &camera);
        if (yagi::calc2PointDistance(point, projected_point) < 10){
            foot_area_vertices_id.push_back(i);
        }
    }
    return foot_area_vertices_id;
}

set<int> FootPrint::findFootPrintAreaIn3DSet(cv::Point2f point, Camera* camera, cv::Mat image){
    set<int> foot_area_vertices_id;

    for (int i = 0; i < this->model.vertices_num; i++) {
        cv::Point2f projected_point = this->worldPointToImagePoint(this->model.vertices[i], camera);
//        cv::circle(image, projected_point, 2, cv::Scalar(255,0,0), 2);
        float dist = yagi::calc2PointDistance(point, projected_point);
        if (dist < this->DIST_RANGE && dist != 0.0){
            foot_area_vertices_id.insert(i);
        }
    }
//    cv::imshow("a", image);
//    cv::waitKey();
    return foot_area_vertices_id;
}

int FootPrint::savePointClouds(){
    //plyfile出力
    cout << "[outputting ply file]:" << endl;

    //header
    string file_name;
//    if (this->FACE_PLY)
//        file_name= this->_projects_path + "/result/result_mesh.ply";
//    else

    if(color == cv::Scalar(0, 255, 0)) {
        file_name = this->_projects_path + "result/extracted_L.ply";
        cout << file_name << endl;
    }else {
        file_name = this->_projects_path + "result/extracted_R.ply";
        cout << file_name << endl;
    }
    ofstream outputfile(file_name);
    outputfile << "ply" << endl;
    outputfile << "format ascii 1.0" << endl;
    outputfile << "comment VCGLIB generated" << endl;
    outputfile << "element vertex " + to_string(this->model.vertices.size()) << endl;
    outputfile << "property float x" << endl;
    outputfile << "property float y" << endl;
    outputfile << "property float z" << endl;
    outputfile << "property uchar red" << endl;
    outputfile << "property uchar green" << endl;
    outputfile << "property uchar blue" << endl;
    outputfile << "property uchar alpha" << endl;
    outputfile << "element face 0" << endl;
    outputfile << "property list uchar int vertex_indices" << endl;
    outputfile << "end_header" << endl;

    for (int i = 0; i < this->model.vertices.size(); i++){

        outputfile  << this->model.vertices[i].x << " "
                    << this->model.vertices[i].y << " "
                    << this->model.vertices[i].z << " "
                    << 255 << " "
                    << 255 << " "
                    << 255 << " "
                    << 255 << endl;


    }

//    if (this->FACE_PLY) {
//        for (int i = 0; i < this->model.faces_num; i++) {
//
//            outputfile << "3" << " "
//                       << this->model.faces[i].verticies[0] << " "
//                       << this->model.faces[i].verticies[1] << " "
//                       << this->model.faces[i].verticies[2] << endl;
//
//        }
//    }
    outputfile.close();
    cout << "ply correctly exported" << endl;
    return 1;
}

int FootPrint::paintFootPrint() {
    for (CameraInfo cm : this->CameraInfoList) {
        string camName = "cam" + to_string(cm.camID + 1);
        for(int legID = 0; legID < 2; legID++) {

            //plyfile出力
            cout << "[outputting ply file]:" << endl;

            //header
            string file_name;
            file_name= this->_projects_path + "/results/" + camName + to_string(legID) + ".ply";

            ofstream outputfile(file_name);
            outputfile << "ply" << endl;
            outputfile << "format ascii 1.0" << endl;
            outputfile << "comment VCGLIB generated" << endl;
            outputfile << "element vertex " + to_string(this->model.vertices_num) << endl;
            outputfile << "property float x" << endl;
            outputfile << "property float y" << endl;
            outputfile << "property float z" << endl;
            outputfile << "property uchar red" << endl;
            outputfile << "property uchar green" << endl;
            outputfile << "property uchar blue" << endl;
            outputfile << "property uchar alpha" << endl;
            outputfile << "element face 0" << endl;
            outputfile << "property list uchar int vertex_indices" << endl;
            outputfile << "end_header" << endl;
            for (int i = 0; i < this->model.vertices.size(); i++) {
                for(int imID = 0; imID < cm.imageList.size(); imID++){

                    if(legID == 0) {
                        //投票で接地判定
                        if (cm.voteOfEachPointList[i].LifStepped[imID] == true) {
                            outputfile << to_string(this->model.vertices[i].x) << " "
                                       << to_string(this->model.vertices[i].y) << " "
                                       << to_string(this->model.vertices[i].z) << " "
                                       << 0 << " "
                                       << 0 << " "
                                       << (legID * 128) + 127 << " "
                                       << 255 << endl;
                        } else {
                            outputfile << this->model.vertices[i].x << " "
                                       << this->model.vertices[i].y << " "
                                       << this->model.vertices[i].z << " "
                                       << 255 << " "
                                       << 255 << " "
                                       << 255 << " "
                                       << 255 << endl;
                        }
                    }
                }
            }
            outputfile.close();
        }
    }
    cout << "ply correctly exported" << endl;
    return 1;
}

int FootPrint::paintClickedPoint() {
    //plyfile出力
    cout << "[outputting ply file]:" << endl;

    //header
    string file_name;
    if (this->FACE_PLY)
        file_name= this->_projects_path + "/result/result_clicked_mesh.ply";
    else
        file_name= this->_projects_path + "/result/result_clicked_point.ply";

    ofstream outputfile(file_name);
    outputfile << "ply" << endl;
    outputfile << "format ascii 1.0" << endl;
    outputfile << "comment VCGLIB generated" << endl;
    outputfile << "element vertex " + to_string(this->model.vertices_num) << endl;
    outputfile << "property float x" << endl;
    outputfile << "property float y" << endl;
    outputfile << "property float z" << endl;
    outputfile << "property uchar red" << endl;
    outputfile << "property uchar green" << endl;
    outputfile << "property uchar blue" << endl;
    outputfile << "property uchar alpha" << endl;
    if (this->FACE_PLY)
        outputfile << "element face " + to_string(this->model.faces_num) << endl;
    else
        outputfile << "element face 0" << endl;
    outputfile << "property list uchar int vertex_indices" << endl;
    outputfile << "end_header" << endl;

    for (int i = 0; i < this->model.vertices_num; i++){
//        cout << this->rightFootPrintID.size() << endl;

        //  全足位置投影
//        if( this->rightFootPrintID.find(i) != this->rightFootPrintID.end() )
//        {
//            outputfile  << to_string(this->model.vertices[i].x) << " "
//                        << to_string(this->model.vertices[i].y) << " "
//                        << to_string(this->model.vertices[i].z) << " "
//                        << 0 << " "
//                        << 0   << " "
//                        << 255   << " "
//                        << 255 << endl;
//        }else if (this->leftFootPrintID.find(i) != this->leftFootPrintID.end()){
//            outputfile  << to_string(this->model.vertices[i].x) << " "
//                        << to_string(this->model.vertices[i].y) << " "
//                        << to_string(this->model.vertices[i].z) << " "
//                        << 0 << " "
//                        << 255   << " "
//                        << 0   << " "
//                        << 255 << endl;
//        } else {
//            outputfile  << this->model.vertices[i].x << " "
//                        << this->model.vertices[i].y << " "
//                        << this->model.vertices[i].z << " "
//                        << 255 << " "
//                        << 255 << " "
//                        << 255 << " "
//                        << 255 << endl;
//        }

        //投票で接地判定
        if( this->right_vote[i] >= VOTE_RANGE )
        {
            outputfile  << to_string(this->model.vertices[i].x) << " "
                        << to_string(this->model.vertices[i].y) << " "
                        << to_string(this->model.vertices[i].z) << " "
                        << 0 << " "
                        << 0   << " "
                        << 255   << " "
                        << 255 << endl;
        }else if (this->left_vote[i] >= VOTE_RANGE){
            outputfile  << to_string(this->model.vertices[i].x) << " "
                        << to_string(this->model.vertices[i].y) << " "
                        << to_string(this->model.vertices[i].z) << " "
                        << 0 << " "
                        << 255   << " "
                        << 0   << " "
                        << 255 << endl;
        } else {
            outputfile  << this->model.vertices[i].x << " "
                        << this->model.vertices[i].y << " "
                        << this->model.vertices[i].z << " "
                        << 255 << " "
                        << 255 << " "
                        << 255 << " "
                        << 255 << endl;
        }

    }

    if (this->FACE_PLY) {
        for (int i = 0; i < this->model.faces_num; i++) {

            outputfile << "3" << " "
                       << this->model.faces[i].verticies[0] << " "
                       << this->model.faces[i].verticies[1] << " "
                       << this->model.faces[i].verticies[2] << endl;

        }
    }
    outputfile.close();
    cout << "ply correctly exported" << endl;
    return 1;

}


//            camera_tmp->t.push_back(stof(translation_tmp[0]));
//            camera_tmp->t.push_back(stof(translation_tmp[1]));
//            camera_tmp->t.push_back(stof(translation_tmp[2]));

//            camera_tmp->R.push_back(stof(rotation_tmp[0]));
//            camera_tmp->R.push_back(stof(rotation_tmp[1]));
//            camera_tmp->R.push_back(stof(rotation_tmp[2]));

//void FootPrint::checkCameraParam(){
//    for(pair<cv::Point2f, int> pt: this->clickLegPoints){
//        cv::Point2f normarized_point;
//        normarized_point.x = (960 - this->cameraParam[2])/this->cameraParam[0];
////        normarized_point.x = (pt.first.x - this->cameraParam[2])/this->cameraParam[0];
////        normarized_point.y = (pt.first.y - this->cameraParam[5])/this->cameraParam[4];
//        normarized_point.y = (540 - this->cameraParam[5])/this->cameraParam[4];
//        cout << normarized_point.x << ", " << normarized_point.y << ", " << 1 << endl;
//    }
//}

void FootPrint::savePlytoTxt(int id){
    string file_name;
    string step_frame_file;
    if(id == 0) {
        file_name = this->_projects_path + "/result/extracted_R.txt";
        step_frame_file = this->_projects_path + "/result/RstepFrame.txt";
    }else {
        file_name = this->_projects_path + "/result/extracted_L.txt";
        step_frame_file = this->_projects_path + "/result/LstepFrame.txt";
    }

    ofstream outputfile(file_name);
    ofstream frameOutputfile(step_frame_file);
    for (int i = 0; i < this->model.vertices.size(); i++){
        //投票で接地判定
        frameOutputfile << this->model.vertices[i].x << " "
                    << this->model.vertices[i].y << " "
                    << this->model.vertices[i].z << " ";

        outputfile<< this->model.vertices[i].x << " "
                       << this->model.vertices[i].y << " "
                       << this->model.vertices[i].z << endl;

        if(id == 0) {
            for (int j = 0; j < RstepFrame.size(); j++) {
                if (i == RstepFrame[j].first)
                    frameOutputfile << " " << RstepFrame[j].second << endl;
            }
        }else{
            for (int j = 0; j < LstepFrame.size(); j++) {
                if (i == LstepFrame[j].first)
                    frameOutputfile << " " << LstepFrame[j].second << endl;
            }
        }
    }
}

void FootPrint::printVoteRecord() {
    string file_name;
    file_name = this->_projects_path + "result/RvoteRecord.txt";
    ofstream outputfile(file_name);
    for(int i = 0; i < this->model.vertices.size(); i++){
        outputfile << this->model.vertices[i].x << " "
                   << this->model.vertices[i].y << " "
                   << this->model.vertices[i].z;
        for(int frame : this->RvoteRecord[i].frameNum){
            outputfile << " " << frame;
        }
        outputfile << endl;
    }

    file_name = this->_projects_path + "result/LvoteRecord.txt";
    ofstream outputfile2(file_name);
    for(int i = 0; i < this->model.vertices.size(); i++){
        outputfile2 << this->model.vertices[i].x << " "
                   << this->model.vertices[i].y << " "
                   << this->model.vertices[i].z;
        for(int frame : this->LvoteRecord[i].frameNum){
            outputfile2 << " " << frame;
        }
        outputfile2 << endl;
    }
}

void FootPrint::votedFrameInit() {
    for(cv::Point3f pt:this->model.vertices){
        Vote newVote;
        newVote.vertice = pt;
        this->RvoteRecord.push_back(newVote);
    }
    for(cv::Point3f pt:this->model.vertices){
        Vote newVote;
        newVote.vertice = pt;
        this->LvoteRecord.push_back(newVote);
    }
}

void FootPrint::calculateSteppedFrame(int id){
    string filename;
    if(id == 0){
        filename = "../Data/Projects/" + this->_project_name + "/result/RvoteRecord.txt";
    }else{
        filename = "../Data/Projects/" + this->_project_name + "/result/LvoteRecord.txt";
    }
    std::ifstream ifs(filename);
    std::string str;

    while (getline(ifs, str)) {
        float frame = 0;
        vector<string> words = yagi::split(str, ' ');
        int i = 0;
        int ptID = 0;
        bool ptFound = false;
        for(i = 0; i < this->model.vertices.size() ; i++){
            cv::Point3f pt = this->model.vertices[i];
            if((pt.x == stof(words[0])) && (pt.y == stof(words[1])) && (pt.z == stof(words[2]))){
                ptFound = true;

                for (int j = 3; j < words.size(); j++){
                    frame += stof(words[j]);
                }
                ptID = i;
            }
        }
        frame /= (words.size() - 3);
        if(ptFound) {
            cout << ptID << " " << frame << endl;
            pair<int, float> step(ptID, frame);
            if(id == 0){
                RstepFrame.push_back(step);
            }else{
                LstepFrame.push_back(step);
            }
        }
    }
}


void FootPrint::videoToImage(){
    for (int camID = this->CAMERA_FIRST_ID; camID < this->CAMERA_FIRST_ID + this->CAMERA_NUM; camID++) {
        string videoName = "cam" + to_string(camID);
        videoToImage::videoToImage(
                this->_video_path + videoName + this->VIDEO_TYPE,
                this->_projects_path + "images/" + videoName
        );
    }
};

void FootPrint::detectHumanPose(){
    for (int camID = this->CAMERA_FIRST_ID; camID < this->CAMERA_FIRST_ID + this->CAMERA_NUM; camID++) {
        string videoName = "cam" + to_string(camID);
        vector<cv::Mat> images;
        cout << "loading images..." << endl;
        yagi::loadImage(this->_projects_path + "images/" + videoName + "/imagelist.txt", &images);
        yagi::outputTextFromImage(this->_video_path + videoName + this->VIDEO_TYPE, this->_openPose_path + videoName, images);
    }
};

void FootPrint::loadAllImages(){
    for(int camID = this->CAMERA_FIRST_ID; camID < this->CAMERA_FIRST_ID + this->CAMERA_NUM; camID++) {
        string cameraName = "cam" + to_string(camID);
        CameraInfo cm = this->CameraInfoList[camID - this->CAMERA_FIRST_ID];
        this->loadImages(this->_projects_path + "images/" + cameraName + "/imagelist.txt", cm.imageList, this->LOAD_LIMIT);
        this->loadOpenPoseData(this->_projects_path + "/openPoseData/" + cameraName + "/human_pose_info.txt", cm.imageList, this->LOAD_LIMIT);
        this->CameraInfoList[camID - this->CAMERA_FIRST_ID] = cm;
    }
}

void FootPrint::estimateCameraPose() {
//    this->loadIntrinsicCameraParam();

    int W = 9;
    int H = 6;
    float SCALE = 100.0;

//    cout << "Input Checker Board WIDTH" << endl;
//    cin >> W;
//    cout << "Input Checker Board HEIGHT" << endl;
//    cin >> H;
//    cout << "Input Checker Board SCALE" << endl;
//    cin >> SCALE;

    //チェッカーポイントの座標格納
    vector<cv::Point3f> objectCorners;
    yagi::generatePointClouds(objectCorners, W, H, SCALE);

    //チェッカーボード検出
    vector<vector<cv::Point2f>> detectedCornerList;
    cv::Mat image;
    for (int camID = this->CAMERA_FIRST_ID; camID < this->CAMERA_FIRST_ID + this->CAMERA_NUM; camID++) {
        string cameraName = "gopro" + to_string(camID);
        image = cv::imread(this->_camera_path + cameraName + "/image0000.jpg");
        vector<cv::Point2f> detectedCorners;
        cv::findChessboardCorners(image, cv::Size(W, H), detectedCorners);
        detectedCornerList.push_back(detectedCorners);

        cv::drawChessboardCorners(image, cv::Size(W,H), detectedCorners, true);
        cv::imshow("chess board", image);
        cv::waitKey(0);
    }

    //カメラ位置姿勢推定
    for (int camID = 0; camID <this->CAMERA_NUM; camID++) {
        string cameraName = "gopro" + to_string(camID + this->CAMERA_FIRST_ID);

        //Rt算出
        cv::Mat R, T;
        cv::solvePnP(objectCorners,
                     detectedCornerList[camID],
                     this->CameraInfoList[camID].camera._A,
                     this->CameraInfoList[camID].camera._dist,
                     R,
                     T);
        this->CameraInfoList[camID].camera._R = R;
        this->CameraInfoList[camID].camera._T = T;
        this->CameraInfoList[camID].camera.outputRt(this->_camera_path + cameraName + "/cameraRt.txt");


        //床点群を再投影
        cout << this->model.vertices.size() << endl;
        cout << this->CameraInfoList[camID].camera._A.type() << endl;
        cout << R.type() << endl;
        cout << T.type() << endl;
        cout << this->CameraInfoList[camID].camera._dist.type() << endl;

        cv::projectPoints(this->model.vertices,
                          R,
                          T,
                          this->CameraInfoList[camID].camera._A,
                          this->CameraInfoList[camID].camera._dist,
                          this->CameraInfoList[camID].projPoints);

//    for (int ptID = 0; ptID < cam.projPoints.size(); ptID++){
//        cam.projPoints[ptID].x *= (this->IMAGE_WIDTH / this->ORIGINAL_IMAGE_WIDTH);
//        cam.projPoints[ptID].y *= (this->IMAGE_HEIGHT / this->ORIGINAL_IMAGE_HEIGHT);
//    }

        cv::Mat dummy = cv::imread("/home/yagi/CLionProjects/prismFootPrint/Data/Camera/gopro" + to_string(this->CameraInfoList[camID].camID + this->CAMERA_FIRST_ID) + "/image0000.jpg");
        for(cv::Point2f pt : this->CameraInfoList[camID].projPoints){
            cv::circle(dummy, pt, 2, cv::Scalar(0,255,0), 2);
        }
        cv::imshow("projected points", dummy);
        cv::waitKey();


    }


}

void FootPrint::generatePlaneModel(){
    Model reconstructedModel;
    vector<cv::Point3f> planePoints;
    yagi::generatePointClouds(planePoints, 90, 60, 10);
    reconstructedModel.vertices = planePoints;
    reconstructedModel.savePly(this->_projects_path + "/planePoints.ply");
}

//        //P-Mat求める
//        cv::Mat Pmat = cv::Mat::zeros(3,4,CV_64F);
//        cv::Mat matR;
//        cv::Rodrigues(R, matR);
//        for(int i = 0; i < 3; i++){
//            for(int j = 0; j < 3; j++){
//                Pmat.at<double>(i, j) = matR.at<double>(i, j);
//            }
//            Pmat.at<double>(i, 3) = T.at<double>(0, i);
//        }
//        Pmat = this->CameraInfoList[camID].camera._A * Pmat;
//        this->CameraInfoList[camID].camera._P = Pmat;
//        cout << Pmat << endl;
//        //stereoCalibrate使う方法
//        cv::Mat R, T, E, F;
//
//        cv::TermCriteria criteria{ 10000, 10000, 0.0001 };
//
//        vector<vector<cv::Point3f>> worldPoints;
//        vector<vector<cv::Point2f>> im1Points;
//        vector<vector<cv::Point2f>> im2Points;
//        worldPoints.push_back(objectCorners);
//        im1Points.push_back(detectedCornerList[0]);
//        im2Points.push_back(detectedCornerList[1]);
//        cv::stereoCalibrate(objectCorners,
//                            detectedCornerList[0],
//                            detectedCornerList[camID - this->CAMERA_FIRST_ID],
//                            this->CameraInfoList[0].camera.getA(),
//                            this->CameraInfoList[0].camera.getDist(),
//                            this->CameraInfoList[camID - this->CAMERA_FIRST_ID].camera.getA(),
//                            this->CameraInfoList[camID - this->CAMERA_FIRST_ID].camera.getDist(),
//                            image.size(),
//                            R, E, T, F, CV_CALIB_FIX_INTRINSIC, criteria
//        );

//        cout << "A " << this->CameraInfoList[0].camera._A << endl;
//        cout << "R " << this->CameraInfoList[0].camera._R << endl;
//        cout << "P " << this->CameraInfoList[0].camera._P << endl;

//          cv::stereoCalibrate(worldPoints,
//                            im1Points,
//                            im2Points,
//                            this->CameraInfoList[0].camera._A,
//                            this->CameraInfoList[0].camera._dist,
//                            this->CameraInfoList[camID - this->CAMERA_FIRST_ID].camera._A,
//                            this->CameraInfoList[camID - this->CAMERA_FIRST_ID].camera._dist,
//                            image.size(),
//                            R, E, T, F
//          );

//        this->CameraInfoList[camID - this->CAMERA_FIRST_ID].camera._R = R;
//        cout << "R " << R << endl;
//        cout << "T " << T << endl;
//
//        //P-Matrixの推定
//        cv::Mat R1, R2, P1, P2, Q;
//        cv::Mat T1(3,1, CV_64F);
//        T1.at<double>(0, 0) = T.at<double>(2, 1);
//        T1.at<double>(1, 0) = T.at<double>(0, 2);
//        T1.at<double>(2, 0) = T.at<double>(1, 0);
//        cv::stereoRectify(
//                this->CameraInfoList[0].camera._A,
//                this->CameraInfoList[0].camera._dist,
//                this->CameraInfoList[camID - this->CAMERA_FIRST_ID].camera._A,
//                this->CameraInfoList[camID - this->CAMERA_FIRST_ID].camera._dist,
//                image.size(),
//                R,
//                T1,
//                R1, R2, P1, P2, Q
//        );



        //findEssential使う方法
//        //すべてのカメラパラメータ同じと仮定
//        cv::Mat E = cv::findEssentialMat(detectedCornerList[0],
//                                         detectedCornerList[camID],
//                                         (this->CameraInfoList[0].camera.focalPoint.x +
//                                                 this->CameraInfoList[0].camera.focalPoint.y) / 2,
//                                         this->CameraInfoList[0].camera.centerPoint);
//        cv::Mat R1, R2, T;
//        cv::decomposeEssentialMat(E, R1, R2, T);
//
//        this->CameraInfoList[camID].camera.setR(R1);
//        this->CameraInfoList[camID].camera.setT(T);
//    }
//
//    //外部パラメータ出力
//    cout << "A " << this->CameraInfoList[0].camera._A << endl;
//    cout << "R " << this->CameraInfoList[0].camera._R << endl;
//    cout << "P " << this->CameraInfoList[0].camera._P << endl;
//    cout << "im size " << image.size() << endl;
//    for (int camID = this->CAMERA_FIRST_ID; camID < this->CAMERA_FIRST_ID + this->CAMERA_NUM; camID++) {
//        string cameraName = "gopro" + to_string(camID);
//        this->CameraInfoList[camID - this->CAMERA_FIRST_ID].camera.outputRt(this->_camera_path + cameraName + "/cameraRt.txt");
//        this->CameraInfoList[camID - this->CAMERA_FIRST_ID].camera.outputP(this->_camera_path + cameraName + "/cameraP.txt");
//    }
//};

void FootPrint::reconstruct3Dpose(){
    for(int imID = 0; imID < this->CameraInfoList[0].imageList.size(); imID++ ) {
        for (int camID = 0; camID < this->CAMERA_NUM; camID++) {

            cv::Mat Tdist = this->CameraInfoList[0].camera._T - this->CameraInfoList[camID].camera._T;
            cv::Mat T1(3,1, CV_64F);
            T1.at<double>(0, 0) = Tdist.at<double>(2, 1);
            T1.at<double>(1, 0) = Tdist.at<double>(0, 2);
            T1.at<double>(2, 0) = Tdist.at<double>(1, 0);

            cv::Mat matR;
            cv::Rodrigues(this->CameraInfoList[0].camera._R * this->CameraInfoList[camID].camera._R.inv(), matR);

            cv::Mat R1, R2, P1, P2, Q;
            cv::stereoRectify(
                this->CameraInfoList[0].camera._A,
                this->CameraInfoList[0].camera._dist,
                this->CameraInfoList[camID].camera._A,
                this->CameraInfoList[camID].camera._dist,
                this->CameraInfoList[0].imageList[0].image.size(),
                matR,
                T1,
                R1, R2, P1, P2, Q
            );

            cv::Mat reconstructedPoints;
            cv::triangulatePoints(P1,
                                  P2,
                                  this->CameraInfoList[camID].imageList[imID].persons[0].getBodyCoord(),
                                  this->CameraInfoList[(camID + 1) % 3].imageList[imID].persons[0].getBodyCoord(),
                                  reconstructedPoints
            );
//            cv::triangulatePoints(this->CameraInfoList[camID].camera.getP(),
//                                  this->CameraInfoList[(camID + 1) % 3].camera.getP(),
//                                  this->CameraInfoList[camID].imageList[imID].persons[0].getBodyCoord(),
//                                  this->CameraInfoList[(camID + 1) % 3].imageList[imID].persons[0].getBodyCoord(),
//                                  reconstructedPoints
//            );

            //3次元復元点の保存
            Model reconstructedModel;
            reconstructedModel.loadFrom4DcvMat(reconstructedPoints);
            reconstructedModel.savePly(this->_projects_path + "/planePoints.ply");

            //平面フィッティング
            this->estimateGroundPlane(reconstructedPoints);
        }
    }
}

void FootPrint::estimateGroundPlane(cv::Mat points){
    const int RDIM=3; // 圧縮後3次元

    cv::PCA pca(points, cv::Mat(), CV_PCA_DATA_AS_ROW,RDIM);

//    cv::Vec3b *src = pca.eigenvectors.ptr<float>(0); //j行目の先頭画素のポインタを取得
//    src[i]; //i番目にアクセス

    //平面点群を出力

}

void FootPrint::projectPoints(CameraInfo &cam){
    //床点群を再投影
//    cout << this->model.vertices.size() << endl;
//    cout << cam.camera._A.type() << endl;
//    cout << cv::Mat(cam.camera._Rvec).type() << endl;
//    cout << cv::Mat(cam.camera._Tvec).type() << endl;
//    cout << cam.camera._dist.type() << endl;
//    cout << cv::Mat(cam.camera._Rvec) << endl;
//    cout << cv::Mat(cam.camera._Tvec) << endl;
//    cout << cam.camera._dist << endl;

//    cv::Point3f point(0,0,0);
//    cv::Point3f camera_coord_point(0,0,0);
//    point.x -= cam.camera._Tvec[0];
//    point.y -= cam.camera._Tvec[1];
//    point.z -= cam.camera._Tvec[2];
//
//    cv::Mat tmpR;
//    cv::Mat(cam.camera._Rvec).convertTo(tmpR, CV_32F);
//
//    cv::Mat R;
//    cv::Rodrigues(tmpR, R);
//    cv::Mat R_inv = R.inv();
//    cout << R_inv << endl;
//
////    camera_coord_point.x = R_inv.at<float>(0, 0)*point.x + R_inv.at<float>(0, 1)*point.y + R_inv.at<float>(0, 2)*point.z;
////    camera_coord_point.y = R_inv.at<float>(1, 0)*point.x + R_inv.at<float>(1, 1)*point.y + R_inv.at<float>(1, 2)*point.z;
////    camera_coord_point.z = R_inv.at<float>(2, 0)*point.x + R_inv.at<float>(2, 1)*point.y + R_inv.at<float>(2, 2)*point.z;
//
//    camera_coord_point.x = R_inv.at<float>(0, 0)*point.x + R_inv.at<float>(1, 0)*point.y + R_inv.at<float>(2, 0)*point.z;
//    camera_coord_point.y = R_inv.at<float>(0, 1)*point.x + R_inv.at<float>(1, 1)*point.y + R_inv.at<float>(2, 1)*point.z;
//    camera_coord_point.z = R_inv.at<float>(0, 2)*point.x + R_inv.at<float>(1, 2)*point.y + R_inv.at<float>(2, 2)*point.z;
//
//
//
////    camera_coord_point.x = R[0]*point.x + R[1]*point.y + R[2]*point.z;
////    camera_coord_point.y = R[3]*point.x + R[4]*point.y + R[5]*point.z;
////    camera_coord_point.z = R[6]*point.x + R[7]*point.y + R[8]*point.z;
//
////    camera_coord_point.x - 8.91;
////    camera_coord_point.y - 0.33;
////    camera_coord_point.z + 4.49;
//
////    camera_coord_point = point;
//
//    //カメラ座標系から正規化画像座標系への変換
//    cv::Point2f normalized_point;
//    normalized_point.x = camera_coord_point.x / camera_coord_point.z;
//    normalized_point.y = camera_coord_point.y / camera_coord_point.z;
//
//    //正規化画像座標系から画像座標系への変換
//    cv::Point2f image_point;
////    cout << this->cameraParam[0] << " " << this->cameraParam[2] << endl;
////    cout << this->cameraParam[4] << " " << this->cameraParam[5] << endl;
////    image_point.x = (normalized_point.x * 800) + 960;
////    image_point.y = (normalized_point.y * 800) + 540;
//    cv::Mat tmpA;
//    cam.camera._A.convertTo(tmpA, CV_32F);
//    image_point.x = (normalized_point.x * tmpA.at<float>(0,0)) + tmpA.at<float>(0,2);
//    image_point.y = (normalized_point.y * tmpA.at<float>(1,1)) + tmpA.at<float>(1,2);
//
//    cout << "imp" << image_point << endl;
//
//    cv::Mat tmpT;
//    cv::Mat(cam.camera._Tvec).convertTo(tmpT, CV_32F);
//
//    cv::Mat tmpDist;
//    cam.camera._dist.convertTo(tmpDist, CV_32F);
//
//    cout << "projected 0,0 " << this->model.vertices[0] << endl;
////    cv::projectPoints(srcPoints, -tmpR, trans, tmpA, tmpDist, cam.projPoints);

    cv::projectPoints(this->model.vertices, cv::Mat(cam.camera._Rvec), cv::Mat(cam.camera._Tvec), cam.camera._A,
                      cam.camera._dist, cam.projPoints);

//    for (int ptID = 0; ptID < cam.projPoints.size(); ptID++){
//        cam.projPoints[ptID].x *= (this->IMAGE_WIDTH / this->ORIGINAL_IMAGE_WIDTH);
//        cam.projPoints[ptID].y *= (this->IMAGE_HEIGHT / this->ORIGINAL_IMAGE_HEIGHT);
//    }

    cv::Mat dummy = cv::imread("/home/yagi/CLionProjects/prismFootPrint/Data/Camera/gopro" + to_string(cam.camID + this->CAMERA_FIRST_ID) + "/image0000.jpg");
    for(cv::Point2f pt : cam.projPoints){
        cv::circle(dummy, pt, 2, cv::Scalar(0,255,0), 2);
    }
    cv::imshow("projected points", dummy);
    cv::waitKey();
};

void FootPrint::estimateStepPositions(){
    for(int camID = 0; camID < this->CAMERA_NUM; camID++) {

        string cameraName = "cam" + to_string(camID + this->CAMERA_FIRST_ID);
        CameraInfo *cm = &this->CameraInfoList[camID];

//        this->right_vote.resize(this->model.vertices_num);
//        this->left_vote.resize(this->model.vertices_num);

        this->loadImages(this->_projects_path + "openPoseData/" + cameraName + "/imagelist.txt", cm->imageList, 50);
        this->loadOpenPoseData(this->_projects_path + "openPoseData/" + cameraName + "/human_pose_info.txt",
                               cm->imageList, 50);
        this->votedFrameInit();
        this->trackTargetPerson(cm->imageList);
        this->projectPoints(*cm);
    }
    this->voting();
    this->countVotes();
    this->paintFootPrint();
}