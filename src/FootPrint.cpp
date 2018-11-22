//
// Created by yagi on 18/07/24.
//

#include "FootPrint.h"
#include "videoToImage/videoToImage.h"
#include "pointCloud/Model.h"
#include "basicFunction/basicFunction.h"
#include "openPose/myOpenPose.h"
#include <opencv2/sfm.hpp>

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

    while (getline(ifs, line)) {

        //imshowがうまくいかないときここ原因(下4行をコメントアウト)
//        if (string_size == 0 || (string_size + 1) == line.size()) {
//            line.erase(line.size() - 1);
//        }
//        string_size = line.size();

        //ImageInfoに画像を格納していく
        ImageInfo image_info;
        image_info.image = cv::imread(line);;
        imageInfoList.push_back(image_info);
        image_num++;
        if(image_num == LIMIT)
            break;
    }
    this->IMAGE_NUM = this->CameraInfoList[0].imageList.size();
    this->IMAGE_WIDTH = this->CameraInfoList[0].imageList[0].image.rows;
    this->IMAGE_HEIGHT = this->CameraInfoList[0].imageList[0].image.cols;
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

//void FootPrint::separateBlocks(){
//    for (int i = 0; i < )
//};


int FootPrint::trackTargetPerson(vector<ImageInfo>& imageInfoList){
    vector<cv::Scalar> colors;
    yagi::setColor(&colors);

    cout << "[Track target person]:" << endl;

    int clicked_frame = 0;
    if(SELECT_TRACKER_BY_CLICKING) {
        //最初のフレームで対象人物クリック
        mouseParam mouseEvent;
        string window_name = "click target person";
        cv::namedWindow(window_name, CV_WINDOW_AUTOSIZE);
        cv::setMouseCallback(window_name, runnerClickCallBackFunc, &mouseEvent);
        bool target_selected = false;

        for (ImageInfo im: imageInfoList) {
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

                if (key == 'n') {
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
    }else{ //最もprobabilityが大きい人物をトラッキング対象とする
        float maxProb = 0;
        for (OpenPosePerson hb: imageInfoList[0].persons){
            float sumOfProbability = accumulate(hb._probabilityList.begin(), hb._probabilityList.end(), 0.0);
            if(maxProb < sumOfProbability) {
                tracking_runner_point = hb.getBodyCoord()[0];
                maxProb = sumOfProbability;
            }
        }
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

            if(this->SHOW_TRACKING_RESULT) {
                cv::imshow("targetRunner", im.image);
                cv::waitKey(0);
            }

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

int sumVecElem(cv::Vec<unsigned char, CHANNEL> vec){
    int sum = 0.0;
    for(int i = 0; i < CHANNEL; i++){
        sum += vec[i];
    }
    return sum;
}



void FootPrint::votingToMap(const int x, const int y, const int imID) {
    int elemSize = voteMap.elemSize();
    int dstChannel = imID % VOTE_RANGE;
//    cv::Mat newChannel = cv::Mat::zeros(PLY_BLOCK_WIDTH, PLY_BLOCK_WIDTH, CV_8U);

    for (int i = 0; i < VOTE_RANGE; i++) {
        for (int j = 0; j < VOTE_RANGE; j++) {
            int xIdx = x + i - (VOTE_RANGE/2);
            int yIdx = y + j - (VOTE_RANGE/2);
            voteMap.at<cv::Vec<unsigned char, CHANNEL>>(xIdx , yIdx)[dstChannel] = 1;
            cout << voteMap.at<cv::Vec<unsigned char, CHANNEL>>( xIdx, yIdx ) << endl;
            //全チャンネルが1なら接地判定
            cout << sumVecElem(voteMap.at<cv::Vec<unsigned char, CHANNEL>>(xIdx, yIdx)) << endl;
            if(sumVecElem(voteMap.at<cv::Vec<unsigned char, CHANNEL>>(xIdx, yIdx)) >= FRAME_RANGE){
                stepMap.at<cv::Vec3b>(xIdx, yIdx) = cv::Vec3b(255,0,0);
            }
        }
    }
}

void FootPrint::voting() {
    cout << "Voting..." << endl;
    for (int ptID = 0; ptID < this->model.vertices.size(); ptID++) {
        VoteOfPoint newPointVote;
        this->model.VoteOfPointsList.push_back(newPointVote);

        for (CameraInfo cm : CameraInfoList) {
            VoteOfCamera newCameraVote;
            this->model.VoteOfPointsList[ptID].push_back(newCameraVote);
            votelist LvoteList;
            votelist RvoteList;

            for (ImageInfo im : cm.imageList) {
                for (OpenPosePerson person: im.persons) {
                    if (person.humanID == 1) {
                        cv::Point2f pt = cm.projPoints[ptID];

                        //右足
                        cv::Point2f RlegPt = person.getBodyCoord()[21];
                        RlegPt.x /= (IMAGE_WIDTH / ORIGINAL_IMAGE_WIDTH);
                        RlegPt.y /= (IMAGE_HEIGHT / ORIGINAL_IMAGE_HEIGHT);

                        //左足
                        cv::Point2f LlegPt = person.getBodyCoord()[24];
                        LlegPt.x *= (IMAGE_WIDTH / ORIGINAL_IMAGE_WIDTH);
                        LlegPt.y *= (IMAGE_HEIGHT / ORIGINAL_IMAGE_HEIGHT);

                        float dist;
                        dist = yagi::calc2PointDistance(pt, LlegPt);
                        if ((dist < this->DIST_RANGE) && (dist != 0.0)) {
                            LvoteList.push_back(true);
                        } else {
                            LvoteList.push_back(false);
                        }

                        dist = yagi::calc2PointDistance(pt, RlegPt);
                        if ((dist < this->DIST_RANGE) && (dist != 0.0)) {
                            RvoteList.push_back(true);
                        } else {
                            RvoteList.push_back(false);
                        }
                    }
                }
            }
            this->model.VoteOfPointsList[ptID][cm.camID]._LvoteList = LvoteList;
            this->model.VoteOfPointsList[ptID][cm.camID]._RvoteList = RvoteList;
        }

        if( (ptID + 1) % (this->model.vertices.size()/100) == 0)
            cout << (ptID + 1) / (this->model.vertices.size()/100) << " % " << endl;
    }
}

void FootPrint::countVotes(){
    for (int ptID = 0; ptID < this->model.vertices.size(); ptID++) {
        for (CameraInfo cm : CameraInfoList) {
            votelist& LvoteList = this->model.VoteOfPointsList[ptID][cm.camID]._LvoteList;
            votelist& RvoteList = this->model.VoteOfPointsList[ptID][cm.camID]._RvoteList;
            votelist LifStepped(IMAGE_NUM, false);
            votelist RifStepped(IMAGE_NUM, false);

            for (int frameID = (FRAME_RANGE / 2); frameID < (IMAGE_NUM - FRAME_RANGE / 2); frameID++) {
                int LwithinRangeVoteCount = 0;
                int RwithinRangeVoteCount = 0;

                //近傍フレーム確認し投票
                for (int voteID = (frameID + (FRAME_RANGE / 2)); voteID < (IMAGE_NUM - (FRAME_RANGE / 2)); voteID++) {
                    if (LvoteList[voteID])
                        LwithinRangeVoteCount++;
                    if (RvoteList[voteID])
                        RwithinRangeVoteCount++;
                }
                //投票数としきい値の比較
                if (LwithinRangeVoteCount > VOTE_RANGE)
                    LifStepped[frameID] = true;
//                else
//                    LifStepped.push_back(false);
                if (RwithinRangeVoteCount > VOTE_RANGE)
                    RifStepped[frameID] = true;
//                else
//                    RifStepped.push_back(false);
            }
            this->model.VoteOfPointsList[ptID][cm.camID]._Lstepped = LifStepped;
            this->model.VoteOfPointsList[ptID][cm.camID]._Rstepped = RifStepped;
        }
        if( (ptID + 1) % (this->model.vertices.size()/100) == 0)
            cout << (ptID + 1) / (this->model.vertices.size()/100) << " % " << endl;
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
//    cv::Point3f imPt;
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
////    imPt.x = R.at<float>(0, 0)*point.x + R.at<float>(0, 1)*point.y + R.at<float>(0, 2)*point.z;
////    imPt.y = R.at<float>(1, 0)*point.x + R.at<float>(1, 1)*point.y + R.at<float>(1, 2)*point.z;
////    imPt.z = R.at<float>(2, 0)*point.x + R.at<float>(2, 1)*point.y + R.at<float>(2, 2)*point.z;
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
    cout << "[outputting ply file]:" << endl;

    //ColorRatio
    float colorRatio = 128.0/(IMAGE_NUM/3);

    for (CameraInfo cm : CameraInfoList) {
        string CAM_NAME = "cam" + to_string(cm.camID + 1);
        const int CAMERA_ID = cm.camID;

        for(int i = 0; i < 2; i++) {
            string LEG_NAME = (i == 0) ? "Right" : "Left";
            string FILE_NAME = this->_projects_path + "/results/" + CAM_NAME + "_" + LEG_NAME + ".ply";

            //header
            ofstream outputfile(FILE_NAME);
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

            for (int ptID = 0; ptID < this->model.vertices.size(); ptID++) {
//                votelist ifStepped = (i == 0) ? this->model.VoteOfPointsList[ptID][CAMERA_ID]._Rstepped :
//                                                this->model.VoteOfPointsList[ptID][CAMERA_ID]._Lstepped ;
                votelist ifStepped = (i == 0) ? this->model.VoteOfPointsList[ptID][CAMERA_ID]._RvoteList :
                                                this->model.VoteOfPointsList[ptID][CAMERA_ID]._LvoteList;

                int Rvalue = 255;
                int Gvalue = 255;
                int Bvalue = 255;

                for (int imID = 0; imID < cm.imageList.size(); imID++) {
                    if (ifStepped[imID] == true) {

                        //フレームIDに応じて色変更
                        Rvalue = int(
                                (imID < 2 * (IMAGE_NUM / 3)) ? 0 : 127 + (imID - (IMAGE_NUM / 3)) * colorRatio);
                        Gvalue = int(
                                (imID < (IMAGE_NUM / 3)) ? 0 : 127 + (imID - (IMAGE_NUM / 3)) * colorRatio);
                        Bvalue = int(127 + (imID * colorRatio));
                        if (LEG_NAME == "Right")
                            swap(Gvalue, Bvalue);
                    }
                }

                outputfile << to_string(this->model.vertices[ptID].x) << " "
                           << to_string(this->model.vertices[ptID].y) << " "
                           << to_string(this->model.vertices[ptID].z) << " "
                           << ((Rvalue > 255) ? 255 : Rvalue) << " "
                           << ((Gvalue > 255) ? 255 : Gvalue) << " "
                           << ((Bvalue > 255) ? 255 : Bvalue) << " "
                           << 255 << endl;
            }
            outputfile.close();
            cout << FILE_NAME + " exported" << endl;
        }
    }
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
        this->loadImages(this->_projects_path + "images/" + cameraName + "/imagelist.txt", cm.imageList, this->FINISH_FRAME);
        this->loadOpenPoseData(this->_projects_path + "/openPoseData/" + cameraName + "/human_pose_info.txt", cm.imageList, this->FINISH_FRAME);
        this->CameraInfoList[camID - this->CAMERA_FIRST_ID] = cm;
    }
}

void FootPrint::estimateCameraPose(){
    if(CHECKER_BOARD_CALIBRATION)
        estimateCameraPoseWithCheckerBoard();
    else
        estimateCameraPoseWithClickingPoints();
}

void FootPrint::estimateCameraPoseWithClickingPoints(){

    cv::Mat image;

    //チェッカーポイントの３次元座標格納
    int W = 2;
    int H = 2;
    float SCALE = 100.0;
    vector<cv::Point3f> objectCorners;
    yagi::generatePointClouds(objectCorners, W, H, SCALE);

//    vector<vector<cv::Point2f>> clickedPointsList;
    for (int camID = this->CAMERA_FIRST_ID; camID < this->CAMERA_FIRST_ID + this->CAMERA_NUM; camID++) {
        string cameraName = "gopro" + to_string(camID);
        image = cv::imread(this->_camera_path + cameraName + "/image0000.jpg");

        //最初のフレームで対象人物クリック
        vector<cv::Point2f> clickedPoints;
        mouseParam mouseEvent;
        string window_name = "Click calibration corners";
        cv::namedWindow(window_name, CV_WINDOW_AUTOSIZE);
        cv::setMouseCallback(window_name, runnerClickCallBackFunc, &mouseEvent);
        bool clicked = false;
        int clickNum = 0;
        while (!clicked) {
            cv::imshow(window_name, image);
            int key = cv::waitKey(1);

            if (flag) {
                //click point格納
                flag = false;
                clickNum++;
                cv::circle(image, tracking_runner_point, 2, cv::Scalar(255, 0, 0), 2);
                cv::putText(image, to_string(clickNum), tracking_runner_point, 0, 1.2, cv::Scalar(255,255,255), 1, CV_AA);
                clickedPoints.push_back(tracking_runner_point);
                cout << "clicked_point = " << tracking_runner_point << endl;
            }

            if (key == 'q') {
                clicked = true;
            }
        }
//        clickedPointsList.push_back(clickedPoints);

        //Rt算出
        cv::Mat R, T;
        cv::solvePnP(objectCorners,
                     clickedPoints,
                     this->CameraInfoList[camID].camera._A,
                     this->CameraInfoList[camID].camera._dist,
                     R,
                     T);
        this->CameraInfoList[camID].camera._R = R;
        this->CameraInfoList[camID].camera._T = T;
        this->CameraInfoList[camID].camera.outputRt(this->_camera_path + cameraName + "/cameraRt.txt");

        cv::projectPoints(objectCorners,
                          R,
                          T,
                          this->CameraInfoList[camID].camera._A,
                          this->CameraInfoList[camID].camera._dist,
                          this->CameraInfoList[camID].projPoints);

        cv::Mat dummy = cv::imread("/home/yagi/CLionProjects/prismFootPrint/Data/Camera/gopro" + to_string(this->CameraInfoList[camID].camID + this->CAMERA_FIRST_ID) + "/image0000.jpg");
        for(cv::Point2f pt : this->CameraInfoList[camID].projPoints){
            cv::circle(dummy, pt, 2, cv::Scalar(0,255,0), 2);
        }
//        cv::imshow("projected points", dummy);
//        cv::waitKey();
    }
}


void FootPrint::estimateCameraPoseWithCheckerBoard() {
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
    yagi::generatePointClouds(objectCorners, H, W, SCALE);

    //チェッカーボード検出
//    vector<vector<cv::Point2f>> detectedCornerList;
    cv::Mat image;
    for (int camID = this->CAMERA_FIRST_ID; camID < this->CAMERA_FIRST_ID + this->CAMERA_NUM; camID++) {
        string cameraName = "gopro" + to_string(camID);
        image = cv::imread(this->_camera_path + cameraName + "/image0000.jpg");
        vector<cv::Point2f> detectedCorners;
        cv::findChessboardCorners(image, cv::Size(W, H), detectedCorners);
        detectedCornerList.push_back(detectedCorners);

        cv::drawChessboardCorners(image, cv::Size(W,H), detectedCorners, true);
//        cv::imshow("chess board", image);
//        cv::waitKey(0);
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
//        cout << this->model.vertices.size() << endl;
//        cout << this->CameraInfoList[camID].camera._A.type() << endl;
//        cout << R.type() << endl;
//        cout << T.type() << endl;
//        cout << this->CameraInfoList[camID].camera._dist.type() << endl;

        cv::projectPoints(objectCorners,
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
//        cv::imshow("projected points", dummy);
//        cv::waitKey();


    }


}

void FootPrint::generatePlaneModel(){
    Model reconstructedModel;
    vector<cv::Point3f> planePoints;
    yagi::generatePointClouds(planePoints, 100, 100, 50, -100, -100);
    yagi::generatePointCloudsAsBlocks(planePoints, 100, 100, 50, -100, -100, PLY_BLOCK_WIDTH, PLY_BLOCK_WIDTH);
    reconstructedModel.vertices = planePoints;
    reconstructedModel.savePly(this->_projects_path + "/planePoints.ply");
}

cv::Mat3f FootPrint::generatePointCloudsAsMatrix(const int width, const int dist){
    cv::Mat plane = cv::Mat::zeros(width * 2, width * 2, CV_32FC3);
    for (int i = 0; i <  width * 2; i++) {
        for (int j = 0; j < width * 2; j++) {
            plane.at<cv::Vec3f>(i, j) = cv::Vec3f((-width + i) *dist, (-width + j)*dist, 0);
            this->model.vertices.push_back(cv::Point3f((-width + i)*dist, (-width + j)*dist, 0));

        }
    }
    cv::Mat3f planePt = plane;
    return planePt;
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

void putValueInPmat(cv::Mat*P1, const Camera* camera){
    cv::Mat Rmat;
    cv::Rodrigues(camera->_R, Rmat);
    cv::Mat T = camera->_T;
    cv::Mat A = camera->_A;

    P1->at<double>(0,0) = Rmat.at<double>(0,0)*A.at<double>(0,0) + Rmat.at<double>(2,0)*A.at<double>(0,2);
    P1->at<double>(0,1) = Rmat.at<double>(0,1)*A.at<double>(0,0) + Rmat.at<double>(2,1)*A.at<double>(0,2);
    P1->at<double>(0,2) = Rmat.at<double>(0,2)*A.at<double>(0,0) + Rmat.at<double>(2,2)*A.at<double>(0,2);
    P1->at<double>(0,3) = T.at<double>(0,0)*A.at<double>(0,0) + T.at<double>(0,2)*A.at<double>(0,2);

    P1->at<double>(1,0) = Rmat.at<double>(1,0)*A.at<double>(1,1) + Rmat.at<double>(2,0)*A.at<double>(1,2);
    P1->at<double>(1,1) = Rmat.at<double>(1,1)*A.at<double>(1,1) + Rmat.at<double>(2,1)*A.at<double>(1,2);
    P1->at<double>(1,2) = Rmat.at<double>(1,2)*A.at<double>(1,1) + Rmat.at<double>(2,2)*A.at<double>(1,2);
    P1->at<double>(1,3) = T.at<double>(0,1)*A.at<double>(1,1) + T.at<double>(0,2)*A.at<double>(1,2);

    P1->at<double>(2,0) = Rmat.at<double>(2,0);
    P1->at<double>(2,1) = Rmat.at<double>(2,1);
    P1->at<double>(2,2) = Rmat.at<double>(2,2);
    P1->at<double>(2,3) = T.at<double>(0,2);
//    for(int i = 0; i < 3; i++){
//        for(int j = 0; j < 3; j++){
//            P1->at<float>(i, j) = Rmat.at<double>(i, j);
//        }
//    }
//
//    for(int i = 0; i < 3; i++){
//        P1->at<float>(i, 3) = camera->_T.at<double>(0, i);
//    }
//
//    P1.
}

cv::Point3f estimate3Dcoord(cv::Mat *A, cv::Mat *R, cv::Mat *T, cv::Point2f pt){
    cout << *A << endl;
    cout << *R << endl;
    cout << *T << endl;
    cv::Point3d imPt((pt.x - A->at<double>(0,2)) /A->at<double>(0,0),
                     (pt.y - A->at<double>(1,2)) /A->at<double>(1,1),
                     1.0);

    cv::Mat Rt = cv::Mat::zeros(4,4,CV_64F);
    Rt.at<double>(0,0) = R->at<double>(0, 0);
    Rt.at<double>(0,1) = R->at<double>(0, 1);
    Rt.at<double>(0,2) = R->at<double>(0, 2);
    Rt.at<double>(0,3) = T->at<double>(0, 0);
    Rt.at<double>(1,0) = R->at<double>(1, 0);
    Rt.at<double>(1,1) = R->at<double>(1, 1);
    Rt.at<double>(1,2) = R->at<double>(1, 2);
    Rt.at<double>(1,3) = T->at<double>(0, 1);
    Rt.at<double>(2,0) = R->at<double>(2, 0);
    Rt.at<double>(2,1) = R->at<double>(2, 1);
    Rt.at<double>(2,2) = R->at<double>(2, 2);
    Rt.at<double>(2,3) = T->at<double>(0, 2);
    Rt.at<double>(3,0) = 0;
    Rt.at<double>(3,1) = 0;
    Rt.at<double>(3,2) = 0;
    Rt.at<double>(3,3) = 1;
    cout << Rt << endl;

    cv::Mat RtInv = Rt.inv();
    cout << RtInv << endl;

    double Z = (imPt.x * RtInv.at<double>(2,0) + imPt.y * RtInv.at<double>(2,1) + imPt.z * RtInv.at<double>(2,2)) ;
    double k = -(RtInv.at<double>(2,3)/Z);

    double X = (imPt.x * RtInv.at<double>(0,0) + imPt.y * RtInv.at<double>(0,1) + imPt.z * RtInv.at<double>(0,2)) ;
    double Y = (imPt.x * RtInv.at<double>(1,0) + imPt.y * RtInv.at<double>(1,1) + imPt.z * RtInv.at<double>(1,2)) ;

    X = k*X + RtInv.at<double>(0,3);
    Y = k*Y + RtInv.at<double>(1,3);

    cv::Point3f pt3D(X, Y, 0.0);
    return pt3D;
};

void FootPrint::vote(Camera* cm, cv::Point2f pt, const int imID){
    cv::Point3f planePt = estimate3Dcoord(&cm->_A, &cm->_Rmat, &cm->_T, pt);
    int xIdx = int(planePt.x / 10);
    int yIdx = int(planePt.y / 10);
    votingToMap(xIdx, yIdx, imID);
};



void FootPrint::estimateStepPositions(){
    for(int imID = 0; imID < this->FINISH_FRAME; imID++){
        for (int camID = 0; camID < this->CAMERA_NUM; camID++) {
            Camera cm = this->CameraInfoList[camID].camera;
            string cameraName = "gopro" + to_string(camID + CAMERA_FIRST_ID);
            cv::Mat image = this->CameraInfoList[camID].imageList[imID].image;
            for (int personID = 0; personID < this->CameraInfoList[camID].imageList[imID].persons.size(); personID++) {
                OpenPosePerson person = this->CameraInfoList[camID].imageList[imID].persons[personID];
                if (person.humanID == 1) {
                    vote(&cm, person.getBodyCoord()[21], imID);
                    vote(&cm, person.getBodyCoord()[24], imID);
//                    cv::Point3f Rpt = estimate3Dcoord(&cm._A, &cm._Rinv, &cm._T, person.getBodyCoord()[21]);
//                    cv::Point3f Lpt = estimate3Dcoord(&cm._A, &cm._Rinv, &cm._T, person.getBodyCoord()[25]);
//                    cv::Point3f Rpt = estimate3Dcoord(&cm._A, &cm._Rmat, &cm._T, detectedCornerList[0][i]);
//                    cv::Point3f Lpt = estimate3Dcoord(&cm._A, &cm._Rmat, &cm._T, person.getBodyCoord()[25]);
//                    int xIdx = int(Rpt.x / 10);
//                    int yIdx = int(Rpt.y / 10);
                }
            }
            cv::imshow("im", stepMap);
            cv::waitKey();
        }
    }
};

void FootPrint::reconstruct3Dpose(){

//    // 3Dを表示するWindow生成
//    string winname = "Viz Camera Pose";
//    cv::viz::Viz3d myWindow(winname);
//
//    // 画面に座標軸を表示
//    myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());

    vector<cv::Mat> projectionMatrixList;

    cv::Mat P1 = cv::Mat::ones(3,4, CV_64F);
    putValueInPmat(&P1, &this->CameraInfoList[0].camera);

//    cv::Mat P2 = cv::Mat::ones(3,4, CV_64F);
//    putValueInPmat(&P2, &this->CameraInfoList[1].camera);
//
//    cv::Mat P3 = cv::Mat::ones(3,4, CV_64F);
//    putValueInPmat(&P3, &this->CameraInfoList[2].camera);
//
//    projectionMatrixList.push_back(P1);
//    projectionMatrixList.push_back(P2);
//    projectionMatrixList.push_back(P3);

//    vector<cv::Mat> floorImagePoints;
//    cv::Mat floorPoints;
//        for (int camID = 0; camID < this->CAMERA_NUM ; camID++) {
//            cv::Mat reshapedTemp = cv::Mat::zeros(2, 54, CV_64F);
//            cv::Mat doubleTemp;
//            cv::Mat(detectedCornerList[camID]).convertTo(doubleTemp, CV_64F);
//            for (int col = 0; col < reshapedTemp.cols; col++) {
//                for (int row = 0; row < reshapedTemp.rows; row++) {
//                    reshapedTemp.at<double>(row, col) = doubleTemp.at<double>(col, row);
//                }
//            }
//            floorImagePoints.push_back(reshapedTemp);
//       }
//    cv::sfm::triangulatePoints(floorImagePoints, projectionMatrixList, floorPoints);
//    cv::Mat floor_cloud_mat(54, 1, CV_64FC3);
//
//    for(int i = 0; i < floorPoints.cols; i++){
//        cv::Vec3d pt(floorPoints.at<double>(0,i), floorPoints.at<double>(1,i), floorPoints.at<double>(2,i));
//        cout << pt << endl;
//        floor_cloud_mat.at<cv::Vec3d>(i,0) = pt;
//    }

    for(int imID = 0; imID < this->IMAGE_NUM; imID++ ) {
        cv::Mat cloud_mat(3, 18, CV_64FC3);
        for (int bdPartID = 0; bdPartID < 25; bdPartID++) {
            if(bdPartID == 21) {
                cv::Mat points;
                vector<cv::Mat> imagePoints;
                vector<cv::Mat> usablePmatList;
                for (int camID = 0; camID < this->CAMERA_NUM; camID++) {
                    string cameraName = "gopro" + to_string(camID + CAMERA_FIRST_ID);
                    cv::Mat image = this->CameraInfoList[camID].imageList[imID].image;
                    for (int personID = 0; personID < this->CameraInfoList[camID].imageList[imID].persons.size(); personID++) {
                        OpenPosePerson person = this->CameraInfoList[camID].imageList[imID].persons[personID];
                        if (person.humanID == 1) {
//                            if (person.getBodyCoord()[bdPartID] != cv::Point2f(0, 0)) {
//                                cv::Mat reshapedTemp = cv::Mat::zeros(2, 1, CV_64F);
//                                reshapedTemp.at<double>(0, 0) = person.getBodyCoord()[bdPartID].x;
//                                reshapedTemp.at<double>(0, 1) = person.getBodyCoord()[bdPartID].y;
//
//                                cv::circle(image, person.getBodyCoord()[bdPartID], 10, cv::Scalar(255, 0, 0), 10);
//
//                                imagePoints.push_back(reshapedTemp);
//                                usablePmatList.push_back(projectionMatrixList[camID]);
//                            }
//                            double x = double(person.getBodyCoord()[bdPartID].x);
//                            double y = double(person.getBodyCoord()[bdPartID].y);
                            double x = double(detectedCornerList[0][0].x);
                            double y = double(detectedCornerList[0][0].y);
                            cv::Point3d pt(x*P1.at<double>(0,0) + y*P1.at<double>(1,0) + P1.at<double>(2,0),
                                        (x*P1.at<double>(0,1) + y*P1.at<double>(1,1) + P1.at<double>(2,1)),
                                        (x*P1.at<double>(0,2) + y*P1.at<double>(1,2) + P1.at<double>(2,2)));
                            pt/=(x*P1.at<double>(0,3) + y*P1.at<double>(1,3) + P1.at<double>(2,3));
//                            pt.x +
//                            cout << pt << endl;
                        }
                    }
                    cv::imshow("im", image);
                    cv::waitKey();
                }


                for (int i = 0; i < imagePoints.size(); i++) {
                    cout << imagePoints[i] << endl;
                    cout << usablePmatList[i] << endl;
                }
                cout << endl;

                if (usablePmatList.size() >= 3) {
                    cv::sfm::triangulatePoints(imagePoints, projectionMatrixList, points);
                    cout << "3Dpt " << points << endl;

                    cv::Vec3d pt(points.at<double>(0, 0), points.at<double>(1, 0), points.at<double>(2, 0));
                    cloud_mat.at<cv::Vec3d>(0, bdPartID) = pt;
                    cout << "3Dpt " << pt << endl;
                }
            }
        }
        // 点群ウィジェット（白）を作成
//        cv::viz::WCloud bdCloud(cloud_mat, cv::viz::Color::blue());
////        cv::viz::WCloud Cloud(floor_cloud_mat, cv::viz::Color::green());
//
//        // 点群ウィジェットを画面へ追加
//        myWindow.showWidget("bd3Dcoords",bdCloud);
////        myWindow.showWidget("bd3Dcos",Cloud);
//
//        // 表示
//        myWindow.spin();
    }

//    for(int imID = 0; imID < this->IMAGE_NUM; imID++ ) {
//        cv::Mat cloud_mat(3, 18, CV_64FC3);
//        for(int i = 0; i < detectedCornerList[0].size(); i++){
//            cv::Mat points;
//            vector<cv::Mat> imagePoints;
//            vector<cv::Mat> usablePmatList;
//            for (int camID = 0; camID < this->CAMERA_NUM; camID++) {
////                string cameraName = "gopro" + to_string(camID + CAMERA_FIRST_ID);
////                for (int personID = 0; personID < this->CameraInfoList[camID].imageList[imID].persons.size(); personID++) {
////                    OpenPosePerson person = this->CameraInfoList[camID].imageList[imID].persons[personID];
////                    if (person.humanID == 1) {
////                        if(person.getBodyCoord()[bdPartID] != cv::Point2f(0,0)) {
////                            cv::Mat reshapedTemp = cv::Mat::zeros(2, 1, CV_64F);
////                            reshapedTemp.at<double>(0, 0) = person.getBodyCoord()[bdPartID].x;
////                            reshapedTemp.at<double>(1, 0) = person.getBodyCoord()[bdPartID].y;
////
//////                            cout << reshapedTemp << endl;
////                            imagePoints.push_back(reshapedTemp);
////                            usablePmatList.push_back(projectionMatrixList[camID]);
////                        }
////                    }
////                }
//                cv::Mat reshapedTemp = cv::Mat::zeros(2, 1, CV_64F);
//                reshapedTemp.at<double>(0, 0) = detectedCornerList[camID][i].x;
//                reshapedTemp.at<double>(1, 0) = detectedCornerList[camID][i].y;
//                imagePoints.push_back(reshapedTemp);
//                usablePmatList.push_back(projectionMatrixList[camID]);
//
//            }
//
//            for(int i = 0; i < imagePoints.size(); i++) {
//                cout << imagePoints[i] << endl;
//                cout << usablePmatList[i] << endl;
//            }
//            cout << endl;
//
//            if(usablePmatList.size() >= 3) {
//                cv::sfm::triangulatePoints(imagePoints, projectionMatrixList, points);
//                cv::Vec3d pt(points.at<double>(0, 0), points.at<double>(0, 1), points.at<double>(0, 2));
//                cloud_mat.at<cv::Vec3d>(0, i) = pt;
//                cout << "3Dpt " << pt << endl;
//            }
//        }

//    // 3Dを表示するWindow生成
//    string winname = "Viz Camera Pose";
//    cv::viz::Viz3d myWindow(winname);
//
//    // 画面に座標軸を表示
//    myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
//
//    vector<cv::Mat> projectionMatrixList;
//
//    cv::Mat P1 = cv::Mat::ones(3,4, CV_64F);
//    putValueInPmat(&P1, &this->CameraInfoList[0].camera);
//
//    cv::Mat P2 = cv::Mat::ones(3,4, CV_64F);
//    putValueInPmat(&P2, &this->CameraInfoList[1].camera);
//
//    cv::Mat P3 = cv::Mat::ones(3,4, CV_64F);
//    putValueInPmat(&P3, &this->CameraInfoList[2].camera);
//
//    projectionMatrixList.push_back(P1);
//    projectionMatrixList.push_back(P2);
//    projectionMatrixList.push_back(P3);
//
//    vector<cv::Mat> floorImagePoints;
//    cv::Mat floorPoints;
//
//    for (int camID = 0; camID < this->CAMERA_NUM - 1; camID++) {
//        cv::Mat reshapedTemp = cv::Mat::zeros(2, 52, CV_64F);
//        cv::Mat doubleTemp;
//        cv::Mat(detectedCornerList[camID]).convertTo(doubleTemp, CV_64F);
//        for (int col = 0; col < reshapedTemp.cols; col++) {
//            for (int row = 0; row < reshapedTemp.rows; row++) {
//                reshapedTemp.at<double>(row, col) = doubleTemp.at<double>(col, row);
//            }
//        }
//        floorImagePoints.push_back(reshapedTemp);
//    }
//    cv::sfm::triangulatePoints(floorImagePoints, projectionMatrixList, floorPoints);
//    cv::Mat floor_cloud_mat(52, 1, CV_64FC3);
//
//    for(int i = 0; i < floorPoints.cols; i++){
//        cv::Vec3d pt(floorPoints.at<double>(0,i), floorPoints.at<double>(1,i), floorPoints.at<double>(2,i));
//        cout << pt << endl;
//        floor_cloud_mat.at<cv::Vec3d>(i,0) = pt;
//    }
//
//    for(int imID = 0; imID < this->IMAGE_NUM; imID++ ) {
//        cv::Mat points;
//        vector<cv::Mat> imagePoints;
//
//
//        for (int camID = 0; camID < this->CAMERA_NUM - 1; camID++) {
//            string cameraName = "gopro" + to_string(camID + CAMERA_FIRST_ID);
//            for(int personID = 0; personID < this->CameraInfoList[camID].imageList[imID].persons.size(); personID++) {
//                OpenPosePerson person =  this->CameraInfoList[camID].imageList[imID].persons[personID];
//                if(person.humanID == 1) {
//                    cv::Mat reshapedTemp = cv::Mat::zeros(2,18, CV_64F);
//                    vector<cv::Point2f> warpedPts;
//                    yagi::mycalcWarpedPoint(person.getBodyCoord(), &warpedPts, imageResizeH.inv());
//                    cv::Mat doubleTemp;
//                    cv::Mat(warpedPts).convertTo(doubleTemp, CV_64F);
//                    for (int col = 0; col < reshapedTemp.cols; col++) {
//                        for (int row = 0; row < reshapedTemp.rows; row++) {
//                            reshapedTemp.at<double>(row, col) = doubleTemp.at<double>(col, row);
//                        }
//                    }
//                    imagePoints.push_back(reshapedTemp);
//                    cout << imagePoints[camID] << endl;
//                }
//            }
//        }
//        cv::sfm::triangulatePoints(imagePoints, projectionMatrixList, points);
//
////        vector<cv::Point2f> projPoints;
////        for (int camID = 0; camID < this->CAMERA_NUM; camID++) {
////            cv::projectPoints(points, cv::Mat(this->CameraInfoList[camID].camera._Rvec), cv::Mat(this->CameraInfoList[camID].camera._Tvec), this->CameraInfoList[camID].camera._A,
////                              this->CameraInfoList[camID].camera._dist, projPoints);
////        }
//        cv::Mat cloud_mat(3, 18, CV_64FC3);
//
//        for(int i = 0; i < points.cols; i++){
////            cout << "correct data: " << objectCorners[i] << endl;
////            cout << points.at<double>(0,i) << " ";
////            cout << points.at<double>(1,i) << " ";
////            cout << points.at<double>(2,i) << endl;
//            cv::Vec3d pt(points.at<double>(0,i), points.at<double>(1,i), points.at<double>(2,i));
//            cout << pt << endl;
//            cloud_mat.at<cv::Vec3d>(i,0) = pt;
//        }
//
//        // 円柱状の点群を作成
////        int N = 10000;
////        float r = 1.0;
//////        cv::Mat cloud_mat(N, 1, CV_32FC3);
////        for (int i = 0;i < N;i++) {
////            float deg = CV_PI / 90 * (i % 180);
////
////            pt(0) = r * std::cos(deg);
////            pt(1) = r * std::sin(deg);
////            pt(2) = r * CV_PI / 90 * (i / 180) + 0.5;
////            cloud_mat.at<cv::Vec3f>(i, 0) = pt;
////            cout << cloud_mat.at<cv::Vec3f>(i, 0) << endl;
////        }
//
//        // 点群ウィジェット（白）を作成
//        cv::viz::WCloud wcloud(cloud_mat, cv::viz::Color::white());
//        cv::viz::WCloud floorcloud(floor_cloud_mat, cv::viz::Color::green());
//
//        // 点群ウィジェットを画面へ追加
//        myWindow.showWidget("Cloud0q", floorcloud);
//        myWindow.showWidget("Cloud", wcloud);
//
//        // 表示
//        myWindow.spin();
//
//    }
}

void FootPrint::setImageResizehomography(){
    vector<cv::Point2f> srcPts;
    cv::Point2f srcPt1(0,0);
    cv::Point2f srcPt2(0,ORIGINAL_IMAGE_HEIGHT);
    cv::Point2f srcPt4(ORIGINAL_IMAGE_WIDTH,0);
    cv::Point2f srcPt3(ORIGINAL_IMAGE_WIDTH,ORIGINAL_IMAGE_HEIGHT);
    srcPts.push_back(srcPt1);
    srcPts.push_back(srcPt2);
    srcPts.push_back(srcPt3);
    srcPts.push_back(srcPt4);

    vector<cv::Point2f> dstPts;
    cv::Point2f dstPt1(0,0);
    cv::Point2f dstPt2(0,IMAGE_HEIGHT);
    cv::Point2f dstPt4(IMAGE_WIDTH,0);
    cv::Point2f dstPt3(IMAGE_WIDTH,IMAGE_HEIGHT);
    dstPts.push_back(dstPt1);
    dstPts.push_back(dstPt2);
    dstPts.push_back(dstPt3);
    dstPts.push_back(dstPt4);

    cv::Mat mask;
    imageResizeH = cv::findHomography(srcPts, dstPts, mask);
};



//            for(int i = 0; i < reconstructedPoints.cols; i++){
//                float x = reconstructedPoints.at<float>(0, i);
//                float y = reconstructedPoints.at<float>(1, i);
//                float z = reconstructedPoints.at<float>(2, i);
//                float base = reconstructedPoints.at<float>(3, i);
//                cv::Point3f pt(x, y, z);
//                pt = pt/base;
//                cout << pt << endl;
//            }


//            cv::Mat Tdist = this->CameraInfoList[0].camera._T - this->CameraInfoList[camID].camera._T;
//            cv::Mat T1(3,1, CV_64F);
//            T1.at<double>(0, 0) = Tdist.at<double>(2, 1);
//            T1.at<double>(1, 0) = Tdist.at<double>(0, 2);
//            T1.at<double>(2, 0) = Tdist.at<double>(1, 0);
//
//            cv::Mat matR;
//            cv::Rodrigues(this->CameraInfoList[0].camera._R * this->CameraInfoList[camID].camera._R.inv(), matR);
//
//            cv::Mat R1, R2, P1, P2, Q;
//            cv::stereoRectify(
//                this->CameraInfoList[0].camera._A,
//                this->CameraInfoList[0].camera._dist,
//                this->CameraInfoList[camID].camera._A,
//                this->CameraInfoList[camID].camera._dist,
//                this->CameraInfoList[0].imageList[0].image.size(),
//                matR,
//                T1,
//                R1, R2, P1, P2, Q
//            );
//
//            cv::triangulatePoints(P1,
//                                  P2,
//                                  this->CameraInfoList[camID].imageList[imID].persons[0].getBodyCoord(),
//                                  this->CameraInfoList[(camID + 1) % 3].imageList[imID].persons[0].getBodyCoord(),
//                                  reconstructedPoints
//            );
////            cv::triangulatePoints(this->CameraInfoList[camID].camera.getP(),
////                                  this->CameraInfoList[(camID + 1) % 3].camera.getP(),
////                                  this->CameraInfoList[camID].imageList[imID].persons[0].getBodyCoord(),
////                                  this->CameraInfoList[(camID + 1) % 3].imageList[imID].persons[0].getBodyCoord(),
////                                  reconstructedPoints
////            );

//3次元復元点の保存
//            Model reconstructedModel;
//            reconstructedModel.loadFrom4DcvMat(reconstructedPoints);
//            reconstructedModel.savePly(this->_projects_path + "/planePoints.ply");
//
//            //平面フィッティング
//            this->estimateGroundPlane(reconstructedPoints);

void FootPrint::estimateGroundPlane(cv::Mat points){
    const int RDIM=3; // 圧縮後3次元

    cv::PCA pca(points, cv::Mat(), CV_PCA_DATA_AS_ROW,RDIM);

//    cv::Vec3b *src = pca.eigenvectors.ptr<float>(0); //j行目の先頭画素のポインタを取得
//    src[i]; //i番目にアクセス

    //平面点群を出力

}

void FootPrint::projectPoints(CameraInfo &cam){
    cv::Mat pts;
    cv::projectPoints(this->model.vertices, cv::Mat(cam.camera._Rvec), cv::Mat(cam.camera._Tvec), cam.camera._A,
                      cam.camera._dist, cam.projPoints);

    //projPointsをMat_に格納
    for(int i = 0; i < this->PLY_BLOCK_WIDTH; i++){
        for(int j = 0; j < this->PLY_BLOCK_WIDTH; j++){
            cam.projPointsMat(cv::Point(j, i) = cam.projPoints[i*this->PLY_BLOCK_WIDTH + j]);
        }
    }

    if(SHOW_REPROJECT_RESULT) {
        cv::Mat dummy = cv::imread("/home/yagi/CLionProjects/prismFootPrint/Data/Camera/gopro" +
                                   to_string(cam.camID + this->CAMERA_FIRST_ID) + "/image0000.jpg");
        for (cv::Point2f pt : cam.projPoints) {
            cv::circle(dummy, pt, 2, cv::Scalar(0, 255, 0), 2);
        }
//        cv::imshow("projected points", dummy);
//        cv::waitKey();
    }
};

void FootPrint::outputTargetPersonInfo(CameraInfo &cam){
    string camName = "cam" + to_string(cam.camID + this->CAMERA_FIRST_ID);
    int imageID = 0;
    const int PT_RANGE = 18;
    for (ImageInfo im: cam.imageList){
        ofstream outputfile(this->_openPose_path + camName + "/targetInfo/openpose" +to_string(imageID)+ ".csv");
        cout << this->_openPose_path + camName + "/targetInfo/openpose" +to_string(imageID)+ ".csv" << endl;
        for(OpenPosePerson person : im.persons){
            if(person.humanID == 1){
                for(int ptID = 0; ptID < PT_RANGE; ptID++){
                    cv::Point2f pt = person.getBodyCoord()[ptID];
                    outputfile << pt.x <<"," << pt.y << endl;
                }
            }
        }
        imageID++;
    }
}

//void FootPrint::estimateStepPositions(){
//    for(int camID = 0; camID < this->CAMERA_NUM; camID++) {
//        string CAM_NAME = "cam" + to_string(camID + this->CAMERA_FIRST_ID);
//        CameraInfo *cm = &this->CameraInfoList[camID];
//
//        loadImages(this->_projects_path + "openPoseData/" + CAM_NAME + "/imagelist.txt", cm->imageList, FINISH_FRAME);
//        loadOpenPoseData(this->_projects_path + "openPoseData/" + CAM_NAME + "/human_pose_info.txt",
//                               cm->imageList, FINISH_FRAME);
//        trackTargetPerson(cm->imageList);
////        outputTargetPersonInfo(*cm);
//        projectPoints(*cm);
//    }
////    reconstruct3Dpose();
//    voting();
////    countVotes();
//    paintFootPrint();
//}