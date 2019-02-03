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
using namespace yagi;

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

bool ifStepped(vector<bool>vec){
    int numOfTrue = 0;
    for(bool tmp : vec){
        if(tmp)
            numOfTrue++;
    }
    return (numOfTrue == 3);
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
                    cv::circle(im.image, pt, 4, colors[3], 4);
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

cv::Vec3b getPointColor(const int bdID){
    switch(bdID){
        case 0:
            return cv::Vec3b(255,0,0);
        case 1:
            return cv::Vec3b(0,255,0);
        case 2:
            return cv::Vec3b(0,0,255);
        case 3:
            return cv::Vec3b(255,0,0);
        case 4:
            return cv::Vec3b(0,255,0);
        case 5:
            return cv::Vec3b(0,0,255);
        default:
            return cv::Vec3b(0,0,0);
    }
}

void FootPrint::addNewStep(cv::Point2f stepPt, const int imID, const int bdID){
    float distToPrevStep = yagi::calc2PointDistance(prevStep, stepPt);
    float distToPrevPrevStep = yagi::calc2PointDistance(prevPrevStep, stepPt);

    //前の接地点との距離がしきい値超えていれば
    if((distToPrevStep > MIN_STRIDE) && (distToPrevPrevStep > MIN_STRIDE)){
        StepInfo newStep;

        // cv::line(trajectoryMap, prevStep, stepPt, cv::Scalar(0, 0, 255), 1);
        prevPrevStep = prevStep;
        prevStep = stepPt;
        walkingDistance += distToPrevStep;
        numOfSteps++;

        newStep.stepPosition = stepPt;
        newStep.frame = imID;
        newStep.stride = distToPrevStep;
        this->stepInfoList.push_back(newStep);

        stepped = true;
        laststepID = bdID;

    }
}

void FootPrint::initVoteChannel(const int dstChannel, cv::Mat *voteMap){
    cv::Mat_<cv::Vec<unsigned char, CHANNEL>> voteMapP = *voteMap;
    for (int i = 0; i < stepMap.cols; i++) {
        for (int j = 0; j < stepMap.rows; j++) {
            voteMapP(cv::Point(i , j))[(dstChannel + 1) % CHANNEL] = 0;
        }
    }
}

void FootPrint::deletePrevSteps(){
//    if(bdID % 2 == 1) {

//           trajectoryを表示
//        if(SHOW_TRAJECTORY){
//            if(prevProjectedPt.x != 0.0) {
//                cv::Point2f centerOfMass((projectedPt + prevProjectedPt) / 2);
//                cv::line(trajectoryMap, prevCoM, centerOfMass, cv::Scalar(0, 0, 255), 1);
//                prevCoM = centerOfMass;
//            }
//        }
//    }
}

void FootPrint::renewStepFlag(const int bdID, cv::Point2f pt){
    switch(bdID){
        case 19:
            leftStepFlag[0] = true;
            leftFoot.toe1 = pt;
            break;

        case 20:
            leftStepFlag[1] = true;
            leftFoot.toe2 = pt;
            break;

        case 21:
            leftStepFlag[2] = true;
            leftFoot.ankle = pt;
            break;

        case 22:
            rightStepFlag[0] = true;
            rightFoot.toe1 = pt;
            break;

        case 23:
            rightStepFlag[1] = true;
            rightFoot.toe2 = pt;
            break;

        case 24:
            rightStepFlag[2] = true;
            rightFoot.ankle = pt;
            break;

        default:
            break;
    }
}


void FootPrint::votingToMap(const int x, const int y, const int imID, const int bdID) {
    cv::Mat *voteMap = &voteMapList[bdID % 2];
    int dstChannel = imID % CHANNEL;
    int stepedListPointer = imID % VISUALIZE_FRAMES;
    vector<cv::Point2f> stepedPoints = stepedPointList[stepedListPointer];
    cv::Vec3b color = getPointColor(bdID);
    cv::Point2f projectedPt = cv::Point2f(x, y);

    //近傍VOTE_RANGE分に投票
    for (int i = 0; i < VOTE_RANGE; i++) {
        for (int j = 0; j < VOTE_RANGE; j++) {
            int xIdx = x + i - (VOTE_RANGE/2);
            int yIdx = y + j - (VOTE_RANGE/2);
            cv::Point2f stepPt(xIdx, yIdx);
            voteMap->at<cv::Vec<unsigned char, CHANNEL>>(stepPt)[dstChannel] = 1;
            HeatVoteMap.at<float>(stepPt) += 1;
            if(sumVecElem(voteMap->at<cv::Vec<unsigned char, CHANNEL>>(stepPt)) >= STEP_THRESHOLD){
                trajectoryMap.at<cv::Vec3b>(stepPt) = color;
                stepMap.at<cv::Vec3b>(stepPt) = color;
                stepedPoints.push_back(stepPt);
//                addNewStep(stepPt, imID);

                if(xIdx == x && yIdx == y){
                    renewStepFlag(bdID, cv::Point2f(x,y));
                }
            }
        }
    }

    //voteMapの該当チャンネルを初期化
    initVoteChannel(dstChannel, voteMap);

    //stepMapから一定フレーム前の接地点を消去
//    deletePrevSteps();
//    stepedPointList[stepedListPointer] = stepedPoints;
    prevProjectedPt = projectedPt;

}

void FootPrint::voting() {
//    cout << "Voting..." << endl;
//    for (int ptID = 0; ptID < this->model.vertices.size(); ptID++) {
//        VoteOfPoint newPointVote;
//        this->model.VoteOfPointsList.push_back(newPointVote);
//
//        for (CameraInfo cm : CameraInfoList) {
//            VoteOfCamera newCameraVote;
//            this->model.VoteOfPointsList[ptID].push_back(newCameraVote);
//            votelist LvoteList;
//            votelist RvoteList;
//
//            for (ImageInfo im : cm.imageList) {
//                for (OpenPosePerson person: im.persons) {
//                    if (person.humanID == 1) {
//                        cv::Point2f pt = cm.projPoints[ptID];
//
//                        //右足
//                        cv::Point2f RlegPt = person.getBodyCoord()[21];
//                        RlegPt.x /= (IMAGE_WIDTH / ORIGINAL_IMAGE_WIDTH);
//                        RlegPt.y /= (IMAGE_HEIGHT / ORIGINAL_IMAGE_HEIGHT);
//
//                        //左足
//                        cv::Point2f LlegPt = person.getBodyCoord()[24];
//                        LlegPt.x *= (IMAGE_WIDTH / ORIGINAL_IMAGE_WIDTH);
//                        LlegPt.y *= (IMAGE_HEIGHT / ORIGINAL_IMAGE_HEIGHT);
//
//                        float dist;
//                        dist = yagi::calc2PointDistance(pt, LlegPt);
//                        if ((dist < this->DIST_RANGE) && (dist != 0.0)) {
//                            LvoteList.push_back(true);
//                        } else {
//                            LvoteList.push_back(false);
//                        }
//
//                        dist = yagi::calc2PointDistance(pt, RlegPt);
//                        if ((dist < this->DIST_RANGE) && (dist != 0.0)) {
//                            RvoteList.push_back(true);
//                        } else {
//                            RvoteList.push_back(false);
//                        }
//                    }
//                }
//            }
//            this->model.VoteOfPointsList[ptID][cm.camID]._LvoteList = LvoteList;
//            this->model.VoteOfPointsList[ptID][cm.camID]._RvoteList = RvoteList;
//        }
//
//        if( (ptID + 1) % (this->model.vertices.size()/100) == 0)
//            cout << (ptID + 1) / (this->model.vertices.size()/100) << " % " << endl;
//    }
}

void FootPrint::countVotes(){
    for (int ptID = 0; ptID < this->model.vertices.size(); ptID++) {
        for (CameraInfo cm : CameraInfoList) {
            votelist& LvoteList = this->model.VoteOfPointsList[ptID][cm.camID]._LvoteList;
            votelist& RvoteList = this->model.VoteOfPointsList[ptID][cm.camID]._RvoteList;
            votelist LifStepped(IMAGE_NUM, false);
            votelist RifStepped(IMAGE_NUM, false);

            for (int frameID = (STEP_THRESHOLD / 2); frameID < (IMAGE_NUM - STEP_THRESHOLD / 2); frameID++) {
                int LwithinRangeVoteCount = 0;
                int RwithinRangeVoteCount = 0;

                //近傍フレーム確認し投票
                for (int voteID = (frameID + (STEP_THRESHOLD / 2)); voteID < (IMAGE_NUM - (STEP_THRESHOLD / 2)); voteID++) {
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

//    yagi::generatePointClouds(this->model.vertices, stepMap.rows, stepMap.cols, 1);

    //plyfile出力
    cout << "[outputting ply file]:" << endl;

    //header
    string file_name;
//    if (this->FACE_PLY)
//        file_name= this->_projects_path + "/result/result_mesh.ply";
//    else

    file_name = "../Data/Projects/" + this->_project_name + "/plane.ply";
    cout << file_name << endl;

    ofstream outputfile(file_name);
    outputfile << "ply" << endl;
    outputfile << "format ascii 1.0" << endl;
    outputfile << "comment VCGLIB generated" << endl;
    outputfile << "element vertex " + to_string(stepMap.rows * stepMap.cols) << endl;
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

    for(int i = 0; i < stepMap.rows; i++){
        for(int j = 0; j < stepMap.cols; j++){
            outputfile << j << " "
                        << i << " "
                        << 0 << " "
                        << int(stepMap.at<cv::Vec3b>(i,j)[2]) << " "
                        << int(stepMap.at<cv::Vec3b>(i,j)[1]) << " "
                        << int(stepMap.at<cv::Vec3b>(i,j)[0]) << " "
                       << 255 << endl;
        }
    }

//    for (int j = 0; j < this->stepedPointList.size(); j++) {
//        for(cv::Point2f pt : stepedPointList[j]){
//            outputfile << int(pt.x) << " "
//                       << int(pt.y) << " "
//                       << 0 << " "
//                       << 0 << " "
//                       << 0 << " "
//                       << 255 << " "
//                       << 255 << endl;
//        }
//    }
//
//    for (int i = 0; i < this->model.vertices.size(); i++){
//            outputfile << this->model.vertices[i].x << " "
//                       << this->model.vertices[i].y << " "
//                       << this->model.vertices[i].z << " "
//                       << 255 << " "
//                       << 255 << " "
//                       << 255 << " "
//                       << 255 << endl;
//    }



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

        //ディレクトリ作成
        const char *cstr = (this->_projects_path + "images/").c_str();
        if (mkdir(cstr, 0777) == 0) {
            printf("directory correctly generated\n");
        } else {
            printf("directory already exists\n");
        }
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

void FootPrint::estimateCameraPoseWithImage(Camera& cm) {

    cv::Mat image;
    cv::VideoCapture capture(0); // USBカメラのオープン
    capture.set(CV_CAP_PROP_FRAME_WIDTH, 800);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 600);
    while(1){
        capture >> image;
        cv::imshow("checker board", image);
        int k = cv::waitKey(1);
        if(k == 115){
            capture.release();
            break;
        }
    }

    int W = 9;
    int H = 6;
    float SCALE = 100.0;

    //チェッカーポイントの座標格納
    yagi::generatePointClouds(objectCorners, H, W, SCALE);

    //チェッカーボード検出
    vector<cv::Point2f> detectedCorners;
    cv::findChessboardCorners(image, cv::Size(W, H), detectedCorners);
    detectedCornerList.push_back(detectedCorners);
    cv::drawChessboardCorners(image, cv::Size(W,H), detectedCorners, true);
    cv::imshow("chess board", image);
    cv::waitKey(0);

    //カメラ位置姿勢推定
    //Rt算出
    cv::Mat R, T;
    cv::solvePnP(objectCorners,
                 detectedCorners,
                 cm._A,
                 cm._dist,
                 R,
                 T);
    cm._R = R;
    cm._T = T;
    cm.outputRt(this->_camera_path + "/" + _project_name + "/cameraRt.txt");

    vector<cv::Point2f> projectPts;
    cv::projectPoints(objectCorners,
                      R,
                      T,
                      cm._A,
                      cm._dist,
                      projectPts);

    int ptID = 0;
    for(cv::Point2f pt : projectPts){
        if(ptID == 0){
            cv::circle(image, pt, 2, cv::Scalar(255, 0, 0), 2);

        }else {
            cv::circle(image, pt, 2, cv::Scalar(0, 255, 0), 2);
        }
        ptID ++;
    }
    cv::imshow("projected points", image);
    cv::waitKey();
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
    yagi::generatePointCloudsAsBlocks(planePoints, 100, 100, 50, -100, -100, PLANE_WIDTH, PLANE_WIDTH);
    reconstructedModel.vertices = planePoints;
    reconstructedModel.savePly(this->_projects_path + "/planePoints.ply");
}

void FootPrint::DetectTargetPerson(op::Array<float>& poses, vector<OpenPosePerson>& personList, OpenPosePerson& target) {
    int peopleNum = poses.getSize()[0];
    for (int personID = 0; personID < peopleNum; personID++) {
        OpenPosePerson newPerson;
        for (int partID = 0; partID < 25; partID++) {
            cv::Point2f pt(poses[(personID * 75) + (partID * 3)], poses[(personID * 75) + (partID * 3) + 1]);
            newPerson._body_parts_coord.push_back(pt);
            newPerson._probabilityList.push_back(poses[(personID * 75) + (partID * 3)] + 2);
        }
        personList.push_back(newPerson);
    }

    //トラッキング対象人物を決定
    int targetID = 0;
    if (TRACKING_MAX_PROB) {
        float maxProb = 0;
        for (int personID = 0; personID < peopleNum; personID++) {
            personList[personID]._probabilityList;
            float sumOfProbability = float(
                    accumulate(personList[personID]._probabilityList.begin(), personList[personID]._probabilityList.end(),
                               0.0));
            if (maxProb < sumOfProbability) {
                targetID = personID;
                maxProb = sumOfProbability;
            }
        }
    }else{
        float minDist = 10000;
        vector<cv::Point2f> clickPoints;
        string trackingFileName = _projects_path + "/trackingTargetPosition.txt";
        ifstream pointFile(trackingFileName);
        if(pointFile.fail()) {
            yagi::clickPoints(backGroundImage, clickPoints, trackingFileName);
        }else {
            string str;
            vector<string> strList;
            while (getline(pointFile, str))
            {
                cv::Point2f pt;
                strList = yagi::split(str, ' ');
                pt.x = stof(strList[0]);
                pt.y = stof(strList[1]);
                clickPoints.push_back(pt);
            }
        }
        //TODO 複数人のトラッキング
        for (int personID = 0; personID < peopleNum; personID++) {
            float dist = yagi::calc2PointDistance(personList[personID]._body_parts_coord[5], clickPoints[0]);
            if(minDist > dist){
                minDist = dist;
                targetID = personID;
            }
        }
    }
    personList[targetID].humanID = 1;
    target = personList[targetID];
}

void calculateFootCoM(OpenPosePerson &person){
    cv::Point2f rFoot(0,0);
    cv::Point2f lFoot(0,0);
    rFoot += person._body_parts_coord[19];
    rFoot += person._body_parts_coord[20];
    rFoot += person._body_parts_coord[21];
    lFoot += person._body_parts_coord[22];
    lFoot += person._body_parts_coord[23];
    lFoot += person._body_parts_coord[24];
    person.rFoot = rFoot/3;
    person.lFoot = lFoot/3;
}

void getPosesInImage(op::Array<float>& poses, vector<OpenPosePerson>& personList){
    int peopleNum = poses.getSize()[0];
    for(int personID = 0; personID < peopleNum; personID++) {
        OpenPosePerson newPerson;
        for(int partID = 0; partID < 25; partID++) {
            cv::Point2f pt(poses[(personID * 75) + (partID * 3)], poses[(personID * 75) + (partID * 3) + 1]);
            newPerson._body_parts_coord.push_back(pt);
            newPerson._probabilityList.push_back(poses[(personID * 75) + (partID * 3)] + 2);
        }
        calculateFootCoM(newPerson);
        personList.push_back(newPerson);
    }
}

float sumOfDistOfPoints(vector<cv::Point2f> ptList1, vector<cv::Point2f> ptList2){
//    CV_ASSERT(ptList1.size() == ptList2.size());
    float sumDist = 0;
    for(int i = 0; i < ptList1.size(); i++){
        sumDist += yagi::calc2PointDistance(ptList1[i], ptList2[i]);
    }
    sumDist /= ptList1.size();
    return sumDist;
}

void tracking(OpenPosePerson& prevPerson, OpenPosePerson& newTarget, vector<OpenPosePerson>& personList){
    int peopleNum = personList.size();
    int trackingID = 0;
    float minDist = 10000;
    for(int personID = 0; personID < peopleNum; personID++) {
        float distOfCoords = sumOfDistOfPoints(personList[personID]._body_parts_coord, prevPerson._body_parts_coord);
        if(distOfCoords < minDist){
            minDist = distOfCoords;
            trackingID = personID;
        }
    }
    personList[trackingID].humanID = 1;
    newTarget = personList[trackingID];
}

void  VisualizeTarget(OpenPosePerson &target, cv::Mat& image){
    for(cv::Point2f pt: target._body_parts_coord){
        cv::circle(image, pt, 2, cv::Scalar(0,0,255), -1);
    }
//    cv::imshow("Tracking target", image);
//    cv::waitKey(1);
}

void FootPrint::stepFlagInit() {
    for(int i = 0; i < 3; i++) {
        rightStepFlag.push_back(false);
        leftStepFlag.push_back(false);
    }
    rightFoot.ankle = cv::Point2f(0.0);
    rightFoot.toe1 = cv::Point2f(0.0);
    rightFoot.toe2 = cv::Point2f(0.0);
    leftFoot.ankle = cv::Point2f(0.0);
    leftFoot.toe1 = cv::Point2f(0.0);
    leftFoot.toe2 = cv::Point2f(0.0);
}



void FootPrint::loadWebCamPram(Camera& cm){
    cm.loadCameraA(this->_camera_path + _project_name + "/intrinsicParam.txt");
    if(ESTIMATE_RT)
        this->estimateCameraPoseWithImage(cm);//カメラ位置姿勢推定
    else
        cm.loadCameraRt(this->_camera_path + _project_name + "/cameraRt.txt");
}

void FootPrint::InitStepMaps(OpenPosePerson targetPerson){
//    cropStepMap();
    if(USE_HOMOGRAPHY){
        //この6は足関節の数s
        for(int i = 0; i < 6; i++){
            voteMapList.push_back(cv::Mat::zeros(overViewImage.size(), CV_8UC(CHANNEL)));
        }
        stepMap = overViewImage.clone();
//        stepMap = cv::Mat::zeros(overViewImage.size(), CV_8UC3);
        trajectoryMap = overViewImage.clone();
        this->ResultInfo = cv::Mat::zeros(stepMap.rows, stepMap.cols, CV_8UC3);


        for(int i = 0; i < VISUALIZE_FRAMES; i++) {
            vector<cv::Point2f> ptList;
            stepedPointList.push_back(ptList);
        }
    }else{
        this->stepMap = cv::imread(_camera_path + _project_name + "/stepMap.jpg");
        this->trajectoryMap = cv::imread(_camera_path + _project_name + "/trajectoryMap.jpg");

    }
    originalStepMap = stepMap.clone();

//    if(USE_HOMOGRAPHY){
//        //この6は足関節の数
//        for(int i = 0; i < 6; i++){
//            voteMapList.push_back(cv::Mat::zeros(overViewImage.size(), CV_8UC(CHANNEL)));
//        }
//        stepMap = cv::Mat::zeros(overViewImage.size(), CV_8UC3);
//        trajectoryMap = overViewImage.clone();
//
//        for(int i = 0; i < VISUALIZE_FRAMES; i++) {
//            vector<cv::Point2f> ptList;
//            stepedPointList.push_back(ptList);
//        }
//    }else{
//        this->stepMap = cv::imread(_camera_path + _project_name + "/stepMap.jpg");
//        this->trajectoryMap = cv::imread(_camera_path + _project_name + "/trajectoryMap.jpg");
//    }
//    originalStepMap = stepMap.clone();

    prevStep = warpPoint(targetPerson.getBodyCoord()[21], warpH);
    prevPrevStep = warpPoint(targetPerson.getBodyCoord()[24], warpH);

    StepInfo firstStep;
    firstStep.frame = 0;
    firstStep.stride = 0;
    firstStep.speed = 0;
    firstStep.stepPosition = prevStep;
    stepInfoList.push_back(firstStep);

    walkingDistance = 0;
    numOfSteps = 0;
    prevStepFrame = 0;
    prevPrevStepFrame = 0;
    prevCoM = (prevStep + prevPrevStep)/2;
    originalStepMap = stepMap.clone();

    stepFlagInit();

    stepFlagInit();
}

void FootPrint::generateOverViewImage(Camera &cm){
    vector<cv::Point2f> imageCornerPoints;
    vector<cv::Point3f> worldCornerPoints;
    push4_3DPointsToVector(worldCornerPoints,
                        cv::Point3f(-(TARGET_AREA_WIDTH/2)*1000, -(TARGET_AREA_HEIGHT/2)*1000, 0),
                        cv::Point3f((TARGET_AREA_WIDTH/2)*1000, -(TARGET_AREA_HEIGHT/2)*1000, 0),
                        cv::Point3f((TARGET_AREA_WIDTH/2)*1000, (TARGET_AREA_HEIGHT/2)*1000, 0),
                        cv::Point3f(-(TARGET_AREA_WIDTH/2)*1000, (TARGET_AREA_HEIGHT/2)*1000, 0));
    cv::projectPoints(worldCornerPoints, cv::Mat(cm._Rvec), cv::Mat(cm._Tvec), cm._A,
                      cm._dist, imageCornerPoints);
//    cv::Rect imTargetAreaRect = obtainRectFrom4Points(imageCornerPoints);
    vector<cv::Point2f> resultCornerPoints;
    push4PointsToVector(resultCornerPoints,
                        cv::Point2f(0,0),
                        cv::Point2f(RESULT_IMAGE_WIDTH * 2, 0),
                        cv::Point2f(RESULT_IMAGE_WIDTH * 2, RESULT_IMAGE_HEIGHT * 2),
                        cv::Point2f(0, RESULT_IMAGE_HEIGHT * 2));
    cv::Mat overviewH = cv::findHomography(imageCornerPoints, resultCornerPoints);
    cv::Mat overViewImage;
    cv::warpPerspective(backGroundImage, overViewImage, overviewH, cv::Size(RESULT_IMAGE_WIDTH, RESULT_IMAGE_HEIGHT));
    this->overViewImage = overViewImage;
    cv::imshow("overView", this->overViewImage);
    cv::waitKey();
}



void FootPrint::estimateStepWithMultipleCameras() {
//カメラパラメータの初期化と読み込み
    this->cameraInfoInit();

//初回のみ行う
//    this->videoToImage();   //動画から画像への変換
//    this->detectHumanPose();    //OpenPoseによる検出
    this->estimateCameraPose();//カメラ位置姿勢推定

//データのロードなど
    for (int camID = 0; camID < this->CAMERA_NUM; camID++) {
        string CAM_NAME = "cam" + to_string(camID + this->CAMERA_FIRST_ID);
        FootPrint::CameraInfo *cm = &this->CameraInfoList[camID];
        this->loadImages(this->_projects_path + "openPoseData/" + CAM_NAME + "/imagelist.txt",
                             cm->imageList, this->FINISH_FRAME);
        this->loadOpenPoseData(this->_projects_path + "openPoseData/" + CAM_NAME + "/human_pose_info.txt",
                                   cm->imageList, this->FINISH_FRAME);
        this->trackTargetPerson(cm->imageList);
        this->projectPoints(*cm);
    }

//接地位置推定
    this->estimateStepPositions();
}

void FootPrint::loadFootImages(){
    leftFootIm = cv::imread(SHOW_IMAGE_PATH + "/left.png");
    rightFootIm = cv::imread(SHOW_IMAGE_PATH + "/right.png");

    cv::resize(leftFootIm, leftFootIm, cv::Size(), FOOTSIZE*10.0/leftFootIm.cols*RESULT_SCALE, FOOTSIZE*10.0/leftFootIm.cols*RESULT_SCALE);
    cv::resize(rightFootIm, rightFootIm, cv::Size(), FOOTSIZE*10.0/rightFootIm.cols*RESULT_SCALE, FOOTSIZE*10.0/rightFootIm.cols*RESULT_SCALE);

    //色をセット
    for(int i = 0; i < leftFootIm.rows; i++){
        for(int j = 0; j < leftFootIm.cols; j++){
            if((leftFootIm.at<cv::Vec3b>(i,j) != cv::Vec3b(0,0,0)) && leftFootIm.at<cv::Vec3b>(i,j) != cv::Vec3b(255,255,255)){
                leftFootIm.at<cv::Vec3b>(i,j) = LEFT_FOOT_COLOR;
            }
            if((rightFootIm.at<cv::Vec3b>(i,j) != cv::Vec3b(0,0,0)) && rightFootIm.at<cv::Vec3b>(i,j) != cv::Vec3b(255,255,255)){
                rightFootIm.at<cv::Vec3b>(i,j) = RIGHT_FOOT_COLOR;
            }
        }
    }

    footImCenter = cv::Point2f(rightFootIm.cols/2, rightFootIm.rows/2);

//    //Affine変換の基準となる3点を決定
//    string fileName = SHOW_IMAGE_PATH + "/affinePoints.txt";
//    ifstream affinePts(fileName);
//    vector<cv::Point2f> clickedPoints;
//    if(affinePts.fail()) {
//        yagi::clickPoints(rightFootIm, clickedPoints, fileName);
//    }else{
//        string str;
//        vector<string> strList;
//        while (getline(affinePts, str))
//        {
//            cv::Point2f pt;
//            strList = yagi::split(str, ' ');
//            pt.x = stof(strList[0]);
//            pt.y = stof(strList[1]);
//            clickedPoints.push_back(pt);
//        }
//    }
//    rightAffinePoints = clickedPoints;
//    leftAffinePoints.push_back(cv::Point2f(rightFootIm.rows - rightAffinePoints[0].x, rightAffinePoints[0].x));
//    leftAffinePoints.push_back(cv::Point2f(rightFootIm.rows - rightAffinePoints[1].x, rightAffinePoints[1].y));
//    leftAffinePoints.push_back(cv::Point2f(rightFootIm.rows - rightAffinePoints[2].x, rightAffinePoints[2].y));
//
//    cv::circle(leftFootIm, leftAffinePoints[0], 2, cv::Scalar(0,0,255), 2);
//    cv::circle(leftFootIm, leftAffinePoints[1], 2, cv::Scalar(255,0,255), 2);
//    cv::circle(leftFootIm, leftAffinePoints[2], 2, cv::Scalar(255,0,255), 2);
//
//    cv::imshow("a", leftFootIm);
//    cv::waitKey();
}

void FootPrint::showAxis(Camera & cm){
    //座標軸をワーピング
    cv::Point3f originPt(0,0,0);
    cv::Point3f xAxis(100,0,0);
    cv::Point3f yAxis(0,100,0);
    vector<cv::Point3f> axisPoints;
    vector<cv::Point2f> axisProjectedPoints;
    axisPoints.push_back(originPt);
    axisPoints.push_back(xAxis);
    axisPoints.push_back(yAxis);
    cv::projectPoints(axisPoints, cv::Mat(cm._Rvec), cv::Mat(cm._Tvec), cm._A,
                      cm._dist, axisProjectedPoints);
    axisProjectedPoints[0].x -= PLANE_WIDTH;
    axisProjectedPoints[0].y -= PLANE_WIDTH;
    axisProjectedPoints[1].x -= PLANE_WIDTH;
    axisProjectedPoints[1].y -= PLANE_WIDTH;
    axisProjectedPoints[2].x -= PLANE_WIDTH;
    axisProjectedPoints[2].y -= PLANE_WIDTH;
    cv::circle(stepMap, axisProjectedPoints[0], 2, cv::Scalar(0,0,255), 2);
    cv::line(stepMap, axisProjectedPoints[0], axisProjectedPoints[1], cv::Scalar(255,0,0), 3);
    cv::line(stepMap, axisProjectedPoints[0], axisProjectedPoints[2], cv::Scalar(0,255,0), 3);
}

void FootPrint::getBackGroundImage() {
    cv::Mat backGround = cv::imread("../Data/Projects/" + _project_name + "/backGround.jpg");
    if (backGround.empty()) {
        cv::VideoCapture capture;
        cv::Mat frame;
        if (USE_WEBCAM) {
            capture.open(0); // USBカメラのオープン
            capture.set(CV_CAP_PROP_FRAME_WIDTH, 800);
            capture.set(CV_CAP_PROP_FRAME_HEIGHT, 600);
        }else{
            capture.open( _projects_path + _project_name + VIDEO_TYPE);
        }
        while (1) {
            capture >> backGround;
            cv::imshow("checker board", backGround);
            int k = cv::waitKey(0);
            if (k == 115) {
                capture.release();
                break;
            }
        }
    }
    cv::imwrite(_projects_path + "/backGround.jpg", backGround);
    this->backGroundImage = backGround;
}

void FootPrint::selectImagePoints(std::vector<cv::Point2f> & clickedPoints){
    string pointFileName = _projects_path + "/cornerPoints.txt";
    ifstream pointFile(pointFileName);
    if(pointFile.fail()) {
        yagi::clickPoints(backGroundImage, clickedPoints, pointFileName);
    }else{
        string str;
        vector<string> strList;
        while (getline(pointFile, str))
        {
            cv::Point2f pt;
            strList = yagi::split(str, ' ');
            pt.x = stof(strList[0]);
            pt.y = stof(strList[1]);
            clickedPoints.push_back(pt);
        }
    }
}

void FootPrint::selectWorldPoints(std::vector<cv::Point2f> & scalePoints){
    ifstream scaleFile(_projects_path + "/scale.txt");
    string str;
    vector<string> strList;
    while (getline(scaleFile, str))
    {
        cv::Point2f pt;
        strList = yagi::split(str, ' ');
        pt.x = stof(strList[0]);
        pt.y = stof(strList[1]);
        pt.x *= RESULT_SCALE;
        pt.y *= RESULT_SCALE;
        scalePoints.push_back(pt);
    }
}

void obtainCheckerBoardPoints( vector<cv::Point2f>& imagePoints,  vector<cv::Point2f>& scalePoints,
                               const int W, const int H, const float SCALE, cv::Mat image, const int AREA_W, const int AREA_H){
    //チェッカーポイントの座標格納
    vector<cv::Point2f> checkerCorners;
    yagi::generatePointCloudsIn2Dscale(checkerCorners, H, W, SCALE, AREA_W, AREA_H);
    scalePoints = checkerCorners;
    vector<cv::Point2f> detectedCorners;
    cv::findChessboardCorners(image, cv::Size(W, H), detectedCorners);
    cv::drawChessboardCorners(image, cv::Size(W,H), detectedCorners, true);
//    cv::imshow("chess board", image);
//    cv::waitKey(0);
    imagePoints = detectedCorners;
}

vector<cv::Point2f> scalingPts(vector<cv::Point2f>& pts, float scale){
    vector<cv::Point2f> scalePts;
    for(cv::Point2f pt: pts){
        pt.x *= scale;
        pt.y *= scale;
        scalePts.push_back(pt);
    }
    return scalePts;
}

void FootPrint::adjustScaleUsingHomography(){
    getBackGroundImage();
    vector<cv::Point2f> imagePoints;
    vector<cv::Point2f> scalePoints;
    cv::Mat image = cv::imread(_projects_path + "/calibrationBoard.jpg");
    if(!image.empty()) {
        int W = 9;
        int H = 6;
        float SCALE = 100.0;
//        backGroundImage = image;
        obtainCheckerBoardPoints(imagePoints, scalePoints, W, H, SCALE, image, TARGET_AREA_WIDTH * 1000, TARGET_AREA_HEIGHT * 1000);
        scalePoints = scalingPts(scalePoints, RESULT_SCALE);
        warpH = cv::findHomography(imagePoints, scalePoints);
        cv::warpPerspective(backGroundImage, overViewImage, warpH, cv::Size(TARGET_AREA_WIDTH * 1000 * 2 * RESULT_SCALE, TARGET_AREA_HEIGHT * 1000 * 2 * RESULT_SCALE));
    }else{
        selectImagePoints(imagePoints);
        selectWorldPoints(scalePoints);
        warpH = cv::findHomography(imagePoints, scalePoints);
        cv::warpPerspective(backGroundImage, overViewImage, warpH, cv::Size(scalePoints[2].x, scalePoints[2].y));
    }
    cv::imshow("backGround", overViewImage);
    cv::waitKey();
};

void FootPrint::estimateStepUsingHomography(){

    adjustScaleUsingHomography();
    cv::VideoCapture capture(_projects_path + _project_name + VIDEO_TYPE);
    cv::Mat frame;

    //OpenPose
    op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};
    opWrapper.start();
    vector<OpenPosePerson> personList;
    OpenPosePerson prevTarget;
    OpenPosePerson newTarget;

    int frameID = 0;
    while (1) {
        capture.read(frame);
        if(!frame.empty()){
            auto datumProcessed = opWrapper.emplaceAndPop(frame);
            frame = datumProcessed->at(0).cvOutputData;
            op::Array<float> poses = datumProcessed->at(0).poseKeypoints;
            if (!poses.getSize().empty()) {
                if (frameID == 0) {
                    firstImage = frame;
                    DetectTargetPerson(poses, personList, newTarget);
                    InitStepMaps(newTarget);
                } else {
                    getPosesInImage(poses, personList);
                    tracking(prevTarget, newTarget, personList);
                    VisualizeTarget(newTarget, frame);
                    EstimateStep(newTarget, frameID);
//                    renewResultInfoIm(frame);
//                    showResultHomography();
                }
            }
            frameID++;
        }else
            break;
        prevTarget = newTarget;
        personList.clear();
//        cv::resize(frame, frame, cv::Size(), IMAGE_WIDTH/frame.cols, IMAGE_HEIGHT/frame.rows);
        cv::imshow("input image", frame);
        cv::imshow("step map", stepMap);
        cv::waitKey(1);
    }
    //可視化処理
}


void FootPrint::estimateStepWithWebCam(){

    //カメラパラメータ読み込み
    Camera cm;
    loadWebCamPram(cm);

    //俯瞰画像生成
    if(SHOW_REPROJECT_RESULT) {
        getBackGroundImage();
        generateOverViewImage(cm);
    }
//        projectPointsForWebCam(cm);

    //動画読み込み
    //    cv::VideoCapture capture(0); // USBカメラのオープン
    cv::VideoCapture capture("/home/yagi/CLionProjects/prismFootPrint/Data/Camera/webCam_oishi/webCam_oishi20181130.webm");
    cv::Mat frame;
    capture.set(CV_CAP_PROP_FRAME_WIDTH, 800);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 600);

    //OpenPose
    op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};
    opWrapper.start();
    vector<OpenPosePerson> personList;
    OpenPosePerson prevTarget;
    OpenPosePerson newTarget;

    int frameID = 0;
    while (1) {
        capture.read(frame);
        auto datumProcessed = opWrapper.emplaceAndPop(frame); //OpenPose
        frame = datumProcessed->at(0).cvOutputData;
        cv::imshow("User worker GUI", frame);
        int k = cv::waitKey(1);
        op::Array<float> poses =  datumProcessed->at(0).poseKeypoints;

        if(!poses.getSize().empty()) { //誰か検出されたら
            if (frameID == 0) {
                DetectTargetPerson(poses, personList, newTarget);
                InitVoteListWebCam(newTarget, cm);
//                InitStepMaps();
                showAxis(cm);

            } else {
                getPosesInImage(poses, personList);
                tracking(prevTarget, newTarget, personList);
                estimateStepAngle(cm, newTarget, frameID);
                VisualizeTarget(newTarget, frame);
                renewResultInfoIm(frame);
                showResult();
            }

            prevTarget = newTarget;
            personList.clear();

            if (k == 27) {
                capture.release();
                cv::destroyAllWindows();
                break;
            }
            frameID++;
        }
    }
}


vector<bool> FootPrint::getFootFlag(const int id){
    if(id == 2)
        return rightStepFlag;
    return leftStepFlag;
}

double estimateDegree(cv::Point2f pt1, cv::Point2f pt2){
    float a, b;
    yagi::getGradSegment(pt1, pt2, &a, &b);

//    cv::Mat debug = cv::Mat::zeros(400,400,CV_8UC3);
//    cv::circle(debug, pt1, 2, cv::Scalar(255,0,0), 2);
//    cv::circle(debug, pt2, 2, cv::Scalar(0,255,0), 2);
//    cv::imshow("aa", debug);
//    cv::waitKey(1);
    double radian = atan(double(a));
    double deg = radian/(PI/180);

    if(pt1.x > pt2.x && pt1.y > pt2.y){
        deg = -deg - 90;
    }else if(pt1.x > pt2.x && pt1.y <= pt2.y){
        deg = -(90 + deg);
    }else if(pt1.x <= pt2.x && pt1.y > pt2.y){
        deg = -(-90 + deg);
    }else{
        deg = 90 - deg;
    }
    cout << endl;
    return deg;
}


double FootPrint::calcAngle(const int footID){
    cv::Vec3b color;
    Foot foot;
    cv::Mat footIm;
    if(footID == 2) {
        foot = rightFoot;
        color = RIGHT_FOOT_COLOR;
        footIm = rightFootIm;
    }
    if(footID == 5) {
        foot = leftFoot;
        color = LEFT_FOOT_COLOR;
        footIm = leftFootIm;
    }

//    float dist = calc2PointDistance(foot.toe2, foot.ankle);
//    float scale = dist/(rightFootIm.rows / 2);

    float deg = estimateDegree(foot.toe2, foot.ankle);
    cv::Mat Rmat = cv::getRotationMatrix2D(footImCenter, deg, 1.0);
    cv::Mat warped;
//    cv::Mat stepMap_Copy = stepMap.clone();

//    float pxSizeInmm = 1/RESULT_SCALE;
//    float scale = FOOTSIZE*10 ;
    cv::warpAffine(footIm, warped, Rmat, footIm.size());
    cv::resize(warped, warped, cv::Size(), RESULT_SCALE,RESULT_SCALE);
    cv::imshow("base", warped);

    for(int i = 0; i < warped.rows; i++){
        for(int j = 0; j < warped.cols; j++){
            if((warped.at<cv::Vec3b>(i,j) == color)){
                cv::Point2f stepMapCoord(j + foot.ankle.x - (footImCenter.x), i + foot.ankle.y - (footImCenter.y));
//                cv::Point2f stepMapCoord(j + foot.ankle.x - (footImCenter.x * scale), i + foot.ankle.y - (footImCenter.y * scale));
                trajectoryMap.at<cv::Vec3b>(stepMapCoord) = color;
            }
        }
    }

//    cv::imshow("stepMap_Copy", trajectoryMap);
    cv::waitKey(1);
    return deg;


//    //３点でアフィン変換
//    cv::Mat footIm;
//    cv::Vec3b footColor;
//    vector<cv::Point2f> imageFootPoints;
//    vector<cv::Point2f> *virtualFootPoints;
//    imageFootPoints.push_back(foot.toe1);
//    imageFootPoints.push_back(foot.toe2);
//    imageFootPoints.push_back(foot.ankle);
//
//    cv::Mat footPrintIm = stepMap.clone();
//
//    if(str == "R") {
//        virtualFootPoints = &rightAffinePoints;
//        footColor = RIGHT_FOOT_COLOR;
//        footIm = rightFootIm;
//    }
//    if(str == "L") {
//        virtualFootPoints = &leftAffinePoints;
//        footColor = LEFT_FOOT_COLOR;
//        footIm = leftFootIm;
//    }
//
//    cv::Mat warpedFootIm;
//    cv::Mat affineH = cv::getAffineTransform(*virtualFootPoints, imageFootPoints);
//    cv::warpAffine(footIm, warpedFootIm, affineH, stepMap.size());
//
//    cv::imshow("wa", warpedFootIm);
//    cv::waitKey();
//
//    for(int i = 0; i < stepMap.rows; i++){
//        for(int j = 0; j < stepMap.cols; j++){
//            if(warpedFootIm.at<cv::Vec3b>(i,j) == footColor) {
//                footPrintIm.at<cv::Vec3b>(i, j) = warpedFootIm.at<cv::Vec3b>(i, j);
//            }
//        }
//    }
//    cv::imshow("s", footPrintIm);
//    cv::waitKey();

//    ２点使って回転角度出す方法
}



void FootPrint::visualizeFootPrint(){
//    if(ifStepped(rightStepFlag)){
//        double angle = calcAngle(rightFoot, "R");
//    }
//    if(ifStepped(leftStepFlag)){
//        double angle = calcAngle(leftFoot, "L");
//    }

}

void FootPrint::estimateStepAngle(Camera& cm, OpenPosePerson& newTarget, const int frameID){
    //right
    vote(&cm, newTarget.getBodyCoord()[22], frameID, 22);
    vote(&cm, newTarget.getBodyCoord()[23], frameID, 23);
    vote(&cm, newTarget.getBodyCoord()[24], frameID, 24);

    //left
    vote(&cm, newTarget.getBodyCoord()[19], frameID, 19);
    vote(&cm, newTarget.getBodyCoord()[20], frameID, 20);
    vote(&cm, newTarget.getBodyCoord()[21], frameID, 21);

    //Flagをチェック（もし2点検出されているなら足あと重畳)
    visualizeFootPrint();
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
//    cout << *A << endl;
//    cout << *R << endl;s
//    cout << *T << endl;
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
//    cout << Rt << endl;

    cv::Mat RtInv = Rt.inv();
//    cout << RtInv << endl;

    double Z = (imPt.x * RtInv.at<double>(2,0) + imPt.y * RtInv.at<double>(2,1) + imPt.z * RtInv.at<double>(2,2)) ;
    double k = -(RtInv.at<double>(2,3)/Z);

    double X = (imPt.x * RtInv.at<double>(0,0) + imPt.y * RtInv.at<double>(0,1) + imPt.z * RtInv.at<double>(0,2)) ;
    double Y = (imPt.x * RtInv.at<double>(1,0) + imPt.y * RtInv.at<double>(1,1) + imPt.z * RtInv.at<double>(1,2)) ;

    X = k*X + RtInv.at<double>(0,3);
    Y = k*Y + RtInv.at<double>(1,3);

    cv::Point3f pt3D(X, Y, 0.0);
    return pt3D;
};

void FootPrint::vote(Camera* cm, cv::Point2f pt, const int imID, const int bdID){
    //3次元位置求める
    cv::Point3f planePt = estimate3Dcoord(&cm->_A, &cm->_Rmat, &cm->_T, pt);

    //画像位置求める
    int xIdx = int(planePt.x / POINT_DIST) + PLANE_WIDTH;
    int yIdx = int(planePt.y / POINT_DIST) + PLANE_WIDTH;

    //座標系変換
    int tmp = xIdx;
    xIdx = PLANE_WIDTH*2 - yIdx;
    yIdx = tmp;

//    cout << xIdx << " " << yIdx << endl;
    if((0 < xIdx && xIdx < 2*PLANE_WIDTH) && (0 < yIdx && yIdx < 2*PLANE_WIDTH))
        votingToMap(xIdx, yIdx, imID, bdID);
};

void FootPrint::voteToStepMap(OpenPosePerson target, const int imID, const int footID){
    //Visualize
    int stepedListPointer = imID % VISUALIZE_FRAMES;
    vector<cv::Point2f> stepedPoints = stepedPointList[stepedListPointer];
    int dstChannel = imID % CHANNEL;

    cv::Point2f pt = (footID == E_RIGHT ? target.rFoot : target.lFoot);
    cv::Mat *voteMap = &voteMapList[footID];
    cv::Vec3b color = getPointColor(footID);
    cv::Point2f warpPt = warpPoint(pt, warpH);

    //近傍VOTE_RANGE分に投票
    for (int i = 0; i < VOTE_RANGE; i++) {
        for (int j = 0; j < VOTE_RANGE; j++) {
            int xIdx = int(warpPt.x + i - (VOTE_RANGE / 2));
            int yIdx = int(warpPt.y + j - (VOTE_RANGE / 2));
            cv::Point2f stepPt(xIdx, yIdx);

            //投影点がVOTEmap内にあれば
            if (xIdx >= 0 && xIdx < voteMap->cols && yIdx >= 0 && yIdx < voteMap->rows) {
                voteMap->at<cv::Vec<unsigned char, CHANNEL>>(stepPt)[dstChannel] = 1;

                //投票数がしきい値超えていればそのpxを接地点とみなす
                if (sumVecElem(voteMap->at<cv::Vec<unsigned char, CHANNEL>>(stepPt)) >= STEP_THRESHOLD) {
                    stepMap.at<cv::Vec3b>(stepPt) = color;
                    stepedPoints.push_back(stepPt);
                }
            }
        }
        //voteMapの該当チャンネルを初期化
        initVoteChannel(dstChannel, voteMap);
    }
//    //stepMapから一定フレーム前の接地点を消去
//    for (cv::Point2f pt : stepedPointList[stepedListPointer]) {
//        stepMap.at<cv::Vec3b>(pt) = originalStepMap.at<cv::Vec3b>(pt);
//    }
    stepedPointList[stepedListPointer] = stepedPoints;
}

void FootPrint::EstimateStep(OpenPosePerson target, const int imID) {
    voteToStepMap(target, imID, E_RIGHT);
    voteToStepMap(target, imID, E_LEFT);
};

void FootPrint::voteForHomography(OpenPosePerson target, const int imID) {

    int stepedListPointer = imID % VISUALIZE_FRAMES;
    vector<cv::Point2f> stepedPoints = stepedPointList[stepedListPointer];
    int dstChannel = imID % CHANNEL;



    for (int bdID = 0; bdID < 6; bdID++)
        if (bdID == 2 || bdID == 5) {
            cv::Point2f pt = target._body_parts_coord[bdID + 19];
            cv::Mat *voteMap = &voteMapList[bdID];
            cv::Vec3b color = getPointColor(bdID + 19);
            cv::Point2f warpPt = warpPoint(pt, warpH);

            //近傍VOTE_RANGE分に投票
            for (int i = 0; i < VOTE_RANGE; i++) {
                for (int j = 0; j < VOTE_RANGE; j++) {
                    int xIdx = int(warpPt.x + i - (VOTE_RANGE / 2));
                    int yIdx = int(warpPt.y + j - (VOTE_RANGE / 2));
                    cv::Point2f stepPt(xIdx, yIdx);

                    //投影点がVOTEmap内にあれば
                    if (xIdx >= 0 && xIdx < voteMap->cols && yIdx >= 0 && yIdx < voteMap->rows) {
                        voteMap->at<cv::Vec<unsigned char, CHANNEL>>(stepPt)[dstChannel] = 1;

                        //投票数がしきい値超えていれば
                        if (sumVecElem(voteMap->at<cv::Vec<unsigned char, CHANNEL>>(stepPt)) >= STEP_THRESHOLD) {
                            stepMap.at<cv::Vec3b>(stepPt) = color;
                            stepedPoints.push_back(stepPt);

                            //かかとの点で歩数としてカウントするか決定
                            if (!stepped) {
                                if (bdID == 2 || bdID == 5) {
                                    if (laststepID != bdID) {
                                        addNewStep(stepPt, imID, bdID);
//                                        calcAngle(laststepID);
                                    }
                                }
                            }

                            //投影点そのものであるとき
                            if (abs(xIdx - int(warpPt.x)) <= 5 && abs(yIdx - int(warpPt.y)) <= 5) {

                                //接地flagを立て接地点も更新
                                renewStepFlag(bdID + 19, cv::Point2f(warpPt.x, warpPt.y));

                            }

                            //新たな接地があり、かつ3点検出
                            if (stepped) {
//                            if (stepped && ifStepped(getFootFlag(laststepID))) {
                                calcAngle(laststepID);
                                stepped = false;
                            }
                        }
                    }
                }
            }
            //voteMapの該当チャンネルを初期化
            initVoteChannel(dstChannel, voteMap);
        }
    //stepMapから一定フレーム前の接地点を消去
    for (cv::Point2f pt : stepedPointList[stepedListPointer]) {
        stepMap.at<cv::Vec3b>(pt) = originalStepMap.at<cv::Vec3b>(pt);
//        trajectoryMap.at<cv::Vec3b>(pt) = originalStepMap.at<cv::Vec3b>(pt);
    }
    stepedPointList[stepedListPointer] = stepedPoints;

    //StepFlagのリセット
    rightStepFlag.clear();
    leftStepFlag.clear();
    stepFlagInit();
//    cv::imshow("stepMap", stepMap);

};

void FootPrint::InitVoteListWebCam(OpenPosePerson& targetPerson, Camera cm) {
    for(int i = 0; i < 2; i++){
        voteMapList.push_back(cv::Mat::zeros(PLANE_WIDTH * 2, PLANE_WIDTH * 2, CV_8UC(CHANNEL)));
    }
    for(int i = 0; i < VISUALIZE_FRAMES; i++) {
        vector<cv::Point2f> ptList;
        stepedPointList.push_back(ptList);
    }

    cv::Point3f pt = estimate3Dcoord(&cm._A, &cm._Rmat, &cm._T, targetPerson.getBodyCoord()[21]);
    pt.x += (PLANE_WIDTH * POINT_DIST);
    pt.y += (PLANE_WIDTH * POINT_DIST);
    prevStep = cv::Point2f(pt.x/10, pt.y/10);
    pt = estimate3Dcoord(&cm._A, &cm._Rmat, &cm._T, targetPerson.getBodyCoord()[24]);
    pt.x += (PLANE_WIDTH * POINT_DIST);
    pt.y += (PLANE_WIDTH * POINT_DIST);
    prevPrevStep = cv::Point2f(pt.x/10, pt.y/10);

    StepInfo firstStep;
    firstStep.frame = 0;
    firstStep.stride = 0;
    firstStep.speed = 0;
    firstStep.stepPosition = prevStep;
    stepInfoList.push_back(firstStep);

    walkingDistance = 0;
    numOfSteps = 0;
    prevStepFrame = 0;
    prevPrevStepFrame = 0;
    prevCoM = (prevStep + prevPrevStep)/2;
    originalStepMap = stepMap.clone();
}

void FootPrint::InitVoteList() {
    for(int i = 0; i < 2; i++){
        voteMapList.push_back(cv::Mat::zeros(PLANE_WIDTH * 2, PLANE_WIDTH * 2, CV_8UC(CHANNEL)));
    }
    for(int i = 0; i < VISUALIZE_FRAMES; i++) {
        vector<cv::Point2f> ptList;
        stepedPointList.push_back(ptList);
    }

    Camera cm = this->CameraInfoList[0].camera;
    for(int i = 0; i < this->CameraInfoList[0].imageList[0].persons.size(); i++) {
        OpenPosePerson person = this->CameraInfoList[0].imageList[0].persons[i];
        if(person.humanID == 1) {
            cv::Point3f pt = estimate3Dcoord(&cm._A, &cm._Rmat, &cm._T, person.getBodyCoord()[21]);
            pt.x += (PLANE_WIDTH * POINT_DIST);
            pt.y += (PLANE_WIDTH * POINT_DIST);
            prevStep = cv::Point2f(pt.x/10, pt.y/10);
            pt = estimate3Dcoord(&cm._A, &cm._Rmat, &cm._T, person.getBodyCoord()[24]);
            pt.x += (PLANE_WIDTH * POINT_DIST);
            pt.y += (PLANE_WIDTH * POINT_DIST);
            prevPrevStep = cv::Point2f(pt.x/10, pt.y/10);

            StepInfo firstStep;
            firstStep.frame = 0;
            firstStep.stride = 0;
            firstStep.speed = 0;
            firstStep.stepPosition = prevStep;
            stepInfoList.push_back(firstStep);
        }
    }
    walkingDistance = 0;
    numOfSteps = 0;
    prevStepFrame = 0;
    prevPrevStepFrame = 0;
    prevCoM = (prevStep + prevPrevStep)/2;

}

void FootPrint::showResult(){
    int imW = PLANE_WIDTH * 2;
    cv::Mat base(imW * 2, imW * 2, CV_8UC3);
    cv::Mat roi1(base, cv::Rect(0 , 0 , imW , imW) );
    cv::Mat trajDummy = trajectoryMap.clone();
    cv::Mat stepDummy = stepMap.clone();
    cv::Mat heatDummy = HeatMap.clone();
    trajDummy.copyTo(roi1);
    cv::Mat roi2( base,cv::Rect(imW , 0 , imW , imW) );
    stepDummy.copyTo(roi2);
    cv::Mat roi3( base,cv::Rect(0 , imW , imW , imW) );
    heatDummy.copyTo(roi3);
    cv::Mat roi4( base,cv::Rect(imW , imW , imW , imW) );
    ResultInfo.copyTo(roi4);

//    cv::line(base, cv::Point2f(0, imW), cv::Point2f(imW*2, imW), cv::Scalar(255,0,0), 2);
//    cv::line(base, cv::Point2f(imW, 0), cv::Point2f(imW, imW*2), cv::Scalar(255,0,0), 2);

    cv::namedWindow("Result",CV_WINDOW_AUTOSIZE);
    cv::resize(stepDummy, stepDummy, cv::Size(), 2,2);
    imshow("Result",base);
}

void FootPrint::showResultHomography(){
    int imW = trajectoryMap.cols;
    int imH = trajectoryMap.rows;
    cv::Mat base(imH, imW * 3, CV_8UC3);
    cv::Mat roi1(base, cv::Rect(0 , 0 , imW , imH) );
    cv::Mat trajDummy = trajectoryMap.clone();
    cv::Mat stepDummy = stepMap.clone();
    cv::Mat heatDummy = HeatMap.clone();
    trajDummy.copyTo(roi1);
    cv::Mat roi2( base,cv::Rect(imW , 0 , imW , imH) );
    stepDummy.copyTo(roi2);
    cv::Mat roi3( base,cv::Rect(imW*2, 0, imW , imW) );
    ResultInfo.copyTo(roi3);
//    cv::Mat roi4( base,cv::Rect(imW , imW , imW , imW) );
//    ResultInfo.copyTo(roi4);

//    cv::line(base, cv::Point2f(0, imW), cv::Point2f(imW*2, imW), cv::Scalar(255,0,0), 2);
//    cv::line(base, cv::Point2f(imW, 0), cv::Point2f(imW, imW*2), cv::Scalar(255,0,0), 2);

    cv::namedWindow("Result",CV_WINDOW_AUTOSIZE);
//    cv::resize(stepDummy, stepDummy, cv::Size(), 2,2);
    cv::resize(base, base, cv::Size(), 0.8, 0.8);
    imshow("Result",base);
}

void FootPrint::renewResultInfoIm(cv::Mat image){
    cv::Mat tmpResultIm = cv::Mat::zeros(ResultInfo.size(), CV_8UC3);
    cv::resize(image, image, cv::Size(), float(ResultInfo.cols)/image.cols, float(ResultInfo.cols)/image.cols);
    cv::Mat roi(tmpResultIm, cv::Rect(0 , 0 , image.cols , image.rows));
    image.copyTo(roi);

    if(numOfSteps > 0) {
        float speed = stepInfoList[numOfSteps].stride/(float(stepInfoList[numOfSteps].frame - stepInfoList[numOfSteps - 1].frame)/VIDEO_FPS);
        stepInfoList[numOfSteps].speed = speed;
    }

    int dotpoint1 = int(to_string(walkingDistance).find("."));
    int dotpoint2 = int(to_string(stepInfoList[numOfSteps].speed).find("."));

    float stringSize = 1.2;
    float lineSpace = 50;
    cv::putText(tmpResultIm, "Walking Distance:" + to_string(walkingDistance/10).substr(0, dotpoint1 + 1) + "cm", cv::Point2f(10, image.rows + lineSpace),  cv::FONT_HERSHEY_SIMPLEX, stringSize, cv::Scalar(255,255,255), 1);
//    cv::putText(tmpResultIm, "Walking Speed:" + to_string(stepInfoList[numOfSteps].speed).substr(0, dotpoint2 + 2) + "cm/sec", cv::Point2f(10, image.rows + (2*lineSpace)),  cv::FONT_HERSHEY_SIMPLEX, stringSize, cv::Scalar(255,255,255), 1);
    cv::putText(tmpResultIm, "Number of Steps:" + to_string(numOfSteps), cv::Point2f(10, image.rows + (3*lineSpace)),  cv::FONT_HERSHEY_SIMPLEX, stringSize, cv::Scalar(255,255,255), 1);

    ResultInfo = tmpResultIm;
}

void FootPrint::estimateStepPositions(){
    InitVoteList();
    for(int imID = 0; imID < this->FINISH_FRAME; imID++){
        for (int camID = 0; camID < this->CAMERA_NUM; camID++) {
            Camera cm = this->CameraInfoList[camID].camera;
            string cameraName = "gopro" + to_string(camID + CAMERA_FIRST_ID);
            cv::Mat image = this->CameraInfoList[camID].imageList[imID].image;
            for (int personID = 0; personID < this->CameraInfoList[camID].imageList[imID].persons.size(); personID++) {
                OpenPosePerson person = this->CameraInfoList[camID].imageList[imID].persons[personID];
                if (person.humanID == 1) {
                    vote(&cm, person.getBodyCoord()[21], imID, 21);
                    vote(&cm, person.getBodyCoord()[24], imID, 24);
                }
            }

            //HeatMap処理
            int votedPt = imID * (VOTE_RANGE * VOTE_RANGE);
            for(int i = 0; i < HeatVoteMap.cols; i++){
                for(int j = 0; j < HeatVoteMap.rows; j++){
                    if (HeatVoteMap.at<float>(j, i) != 0){
//                        cout << (HeatVoteMap.at<float>(j, i)) / votedPt << " " << HeatVoteMap.at<float>(j, i) << " "
//                             << votedPt << endl;
                        int greenValue = 25500*(HeatVoteMap.at<float>(j, i))/votedPt;
                        if (greenValue > 255)
                            greenValue = 255;
                        HeatMap.at<cv::Vec3b>(j, i) = cv::Vec3b(0, greenValue, 0);
                    }
                }
            }
            renewResultInfoIm(this->CameraInfoList[camID].imageList[imID].image);
            showResult();
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
//                    cv::sfm::triangulatePoints(imagePoints, projectionMatrixList, points);
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

void FootPrint::projectPointsForWebCam(Camera &cm){

    cv::Mat im;
    cv::VideoCapture capture(0); // USBカメラのオープン
    capture.set(CV_CAP_PROP_FRAME_WIDTH, 800);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 600);
    while(1){
        capture >> im;
        cv::imshow("checker board", im);
        int k = cv::waitKey(1);
        if(k == 115){
            capture.release();
            break;
        }
    }

    cv::Mat pts;
    vector<cv::Point2f> projectPts;
    cv::Mat2f projectPtsMat;
    cv::projectPoints(this->model.vertices, cv::Mat(cm._Rvec), cv::Mat(cm._Tvec), cm._A,
                      cm._dist, projectPts);

    //projPointsをMat_に格納
    for(int i = 0; i < this->PLANE_WIDTH; i++){
        for(int j = 0; j < this->PLANE_WIDTH; j++){
            projectPtsMat(cv::Point(j, i) = projectPts[i*this->PLANE_WIDTH + j]);
        }
    }

    cv::Mat dummy = im.clone();
    if(SHOW_REPROJECT_RESULT) {
        int ptID = 0;
        for (cv::Point2f pt : projectPts) {
            cv::circle(dummy, pt, 2, cv::Scalar(0, 255, 0), 2);
        }
        cv::imshow("projected points", dummy);
        cv::waitKey();
    }

    //ワーピングのための4点
    vector<cv::Point2f> projPtList;
    projPtList.push_back(projectPts[((PLANE_WIDTH * 2)) - 1]);
    projPtList.push_back(projectPts[0]);
    projPtList.push_back(projectPts[(PLANE_WIDTH * 2) * (PLANE_WIDTH * 2) - (PLANE_WIDTH * 2)]);
    projPtList.push_back(projectPts[(PLANE_WIDTH * 2 * PLANE_WIDTH * 2) - 1]);


    vector<cv::Point2f> planePtList;
    planePtList.push_back(cv::Point2f(0, 0));
    planePtList.push_back(cv::Point2f(PLANE_WIDTH * 2, 0));
    planePtList.push_back(cv::Point2f(PLANE_WIDTH * 2, PLANE_WIDTH * 2));
    planePtList.push_back(cv::Point2f(0, PLANE_WIDTH * 2));

    cv::Mat H = cv::findHomography(planePtList, projPtList);

    cv::Mat warped;
    cv::warpPerspective(im, warped, H.inv(), stepMap.size());

    cv::imshow("warped", warped);
    cv::waitKey();
    cv::imwrite(_camera_path + _project_name + "/stepMap.jpg", warped);
    cv::imwrite(_camera_path + _project_name + "/trajectoryMap.jpg", warped);
}


void FootPrint::projectPoints(CameraInfo &cam){
    cv::Mat pts;
    cv::projectPoints(this->model.vertices, cv::Mat(cam.camera._Rvec), cv::Mat(cam.camera._Tvec), cam.camera._A,
                      cam.camera._dist, cam.projPoints);

    //projPointsをMat_に格納
    for(int i = 0; i < this->PLANE_WIDTH; i++){
        for(int j = 0; j < this->PLANE_WIDTH; j++){
            cam.projPointsMat(cv::Point(j, i) = cam.projPoints[i*this->PLANE_WIDTH + j]);
        }
    }

    cv::Mat dummy = cv::imread("/home/yagi/CLionProjects/prismFootPrint/Data/Camera/gopro" +
                               to_string(cam.camID + this->CAMERA_FIRST_ID) + "/image0000.jpg");

    if(SHOW_REPROJECT_RESULT) {
        int ptID = 0;
        for (cv::Point2f pt : cam.projPoints) {
            cv::circle(dummy, pt, 2, cv::Scalar(0, 255, 0), 2);
        }
        cv::imshow("projected points", dummy);
        cv::waitKey();
    }

    if(PLOT_ON_WARPED_IMAGE) {
        //ワーピングのための4点
        vector<cv::Point2f> projPtList;
        projPtList.push_back(cam.projPoints[0]);
        projPtList.push_back(cam.projPoints[((PLANE_WIDTH * 2)) - 1]);
        projPtList.push_back(cam.projPoints[(PLANE_WIDTH * 2 * PLANE_WIDTH * 2) - 1]);
        projPtList.push_back(cam.projPoints[(PLANE_WIDTH * 2) * (PLANE_WIDTH * 2) - (PLANE_WIDTH * 2)]);


        vector<cv::Point2f> planePtList;
        planePtList.push_back(cv::Point2f(PLANE_WIDTH * 2, 0));
        planePtList.push_back(cv::Point2f(0, 0));
        planePtList.push_back(cv::Point2f(0, PLANE_WIDTH * 2));
        planePtList.push_back(cv::Point2f(PLANE_WIDTH * 2, PLANE_WIDTH * 2));

        cv::Mat H = cv::findHomography(planePtList, projPtList);

        cv::Mat warped;
        cv::warpPerspective(dummy, warped, H.inv(), stepMap.size());
        stepMap = warped.clone();

        trajectoryMap = warped.clone();
//        HeatMap = warped.clone();
    }
    originalStepMap = stepMap.clone();

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