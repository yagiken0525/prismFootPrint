//
// Created by yagi on 18/07/24.
//

#include "FootPrint.h"
#include "./basicFunction/basicFunction.h"

using namespace std;

int FootPrint::loadCameraParam(std::string file_name) {
    std::ifstream ifs(file_name);
    std::string str;
    if (ifs.fail())
    {
        std::cout << "cannot open "<< file_name << std::endl;
        return -1;
    }

    cout << "[Camera parameter correctly loaded]:" << endl;
    vector<string> params;
    while (getline(ifs, str))
    {
        params = yagi::split(str, ' ');
        for(string tmp: params){
            this->cameraParam.push_back(stof(tmp));
        }
    }
    for (float tmp: this->cameraParam){
        cout << tmp << " ";
    }
    cout << endl;
    return 1;
}

int FootPrint::loadImages(string file_path) {

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

        this->image_infos.push_back(image_info);

    }
}


int FootPrint::loadMultipleCameraRts(std::string file_name){
    ifstream ifs(file_name);
    string buf;

    if(!ifs.is_open()){
        std::cout << "cannot open "<< file_name << std::endl;
        return false;
    }

    cout << "[Camera pose file correctly loaded:]" << endl;
    int line_num = 0;
    int camera_num = 0;
    Camera camera_tmp;
    cv::Mat t_tmp = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat r_tmp = cv::Mat::zeros(3, 3, CV_64F);
    while(getline(ifs,buf)){
        if ((line_num % 14) == 5){
            camera_tmp.setId(camera_num);
            vector<string> translation_tmp  = yagi::split(buf, ' ');
            cv::Point3f tmp;
//            t_tmp.at<double>(0, 0) = stof(translation_tmp[0]);
//            t_tmp.at<double>(0, 1) = stof(translation_tmp[1]);
//            t_tmp.at<double>(0, 2) = stof(translation_tmp[2]);
            camera_tmp._T.x = stof(translation_tmp[0]);
            camera_tmp._T.y = stof(translation_tmp[1]);
            camera_tmp._T.z = stof(translation_tmp[2]);
//            camera_tmp._T.at<double>(0,0) = stof(translation_tmp[0]);
//            camera_tmp._T.at<double>(1,0) = stof(translation_tmp[1]);
//            camera_tmp._T.at<double>(2,0) = stof(translation_tmp[2]);
//            cv::Mat tmp = t_tmp.clone();
//            camera_tmp.setT(tmp);

//        }else if((line_num % 14) == 8){
//            vector<string> rotation_tmp  = yagi::split(buf, ' ');
//            r_tmp.at<double>(0, 0) = stof(rotation_tmp[0]);
//            r_tmp.at<double>(0, 1) = stof(rotation_tmp[1]);
//            r_tmp.at<double>(0, 2) = stof(rotation_tmp[2]);
//
//        }else if((line_num % 14) == 9){
//            vector<string> rotation_tmp  = yagi::split(buf, ' ');
//            r_tmp.at<double>(1, 0) = stof(rotation_tmp[0]);
//            r_tmp.at<double>(1, 1) = stof(rotation_tmp[1]);
//            r_tmp.at<double>(1, 2) = stof(rotation_tmp[2]);
//
//        }else if((line_num % 14) == 10){
//            vector<string> rotation_tmp  = yagi::split(buf, ' ');
//            r_tmp.at<double>(2, 0) = stof(rotation_tmp[0]);
//            r_tmp.at<double>(2, 1) = stof(rotation_tmp[1]);
//            r_tmp.at<double>(2, 2) = stof(rotation_tmp[2]);
//            cv::Mat tmp = r_tmp.clone();
//            camera_tmp.setR(tmp);

        }else if((line_num % 14) == 8){
            vector<string> rotation_tmp  = yagi::split(buf, ' ');
//            camera_tmp._R.push_back(stof(rotation_tmp[0]));
//            camera_tmp._R.push_back(stof(rotation_tmp[1]));
//            camera_tmp._R.push_back(stof(rotation_tmp[2]));
            camera_tmp._R.push_back(stof(rotation_tmp[0]));
            camera_tmp._R.push_back(stof(rotation_tmp[1]));
            camera_tmp._R.push_back(stof(rotation_tmp[2]));

        }else if((line_num % 14) == 9){
            vector<string> rotation_tmp  = yagi::split(buf, ' ');
            camera_tmp._R.push_back(stof(rotation_tmp[0]));
            camera_tmp._R.push_back(stof(rotation_tmp[1]));
            camera_tmp._R.push_back(stof(rotation_tmp[2]));

        }else if((line_num % 14) == 10){
            vector<string> rotation_tmp  = yagi::split(buf, ' ');
            camera_tmp._R.push_back(stof(rotation_tmp[0]));
            camera_tmp._R.push_back(stof(rotation_tmp[1]));
            camera_tmp._R.push_back(stof(rotation_tmp[2]));
//            cv::Mat tmp = r_tmp.clone();
//            camera_tmp.setR(tmp);

        }else if((line_num % 14) == 13){
            Camera tmp = camera_tmp;
            this->image_infos[camera_num].camera = tmp;
            camera_num++;
            camera_tmp._R.clear();
        }
        line_num++;
    }
    return 1;
}

int FootPrint::loadOpenPoseData(string file_name){

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

        if (coords[0] == "Frame:"){
            frame_num++;
        }else if (coords[0] == "no"){
            frame_counter++;
        } else if (coords.size() == 5) {

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
    }
    cout << allFramePersons.size() << endl;

    for (int i = 0; i < person_detected_frameID.size(); i++) {
        this->image_infos[person_detected_frameID[i]].persons = allFramePersons[i];
    }
}


//int FootPrint::loadOpenPoseData(string file_name){
//
//    cout << "[Loading open pose data]:" << endl;
//    //OpenPoseのデータ読みこみ
//    ifstream ifs(file_name);
//    if(!ifs.is_open()){
//        std::cout << "cannot open "<< file_name << std::endl;
//        return false;
//    }
//    string line;
//    OpenPosePerson person;
//    vector<OpenPosePerson> persons;
//    vector<vector<OpenPosePerson>> allFramePersons;
//
//    bool first_person = true;
//    int frame_counter = 0;
//
//    while (getline(ifs, line)) {
//
//        vector<string> coords = yagi::split(line, ' ');
//
//        if (coords.size() == 5) {
//
//            if (!first_person) {
//
//                OpenPosePerson dummy_person = person;
//                persons.push_back(dummy_person);
//                person.clearBodyCoord();
//
//                if (coords[1] == "0") {
//                    vector<OpenPosePerson> dummy_persons = persons;
//                    allFramePersons.push_back(dummy_persons);
//
//                    persons.clear();
//                    frame_counter++;
//                }
//            }
//
//        } else {
//            person.setBodyCoord(coords);
//            first_person = false;
//        }
//    }
//    cout << allFramePersons.size() << endl;
//
//    for (int i = 0; i < allFramePersons.size(); i++) {
//        this->image_infos[i].persons = allFramePersons[i];
//    }
//}

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
//            std::cout << x << " , " << y << std::endl;
            tracking_runner_point.x = x;
            tracking_runner_point.y = y;
            flag = true;
    }
}

int FootPrint::trackTargetPerson(){
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


    for(ImageInfo im: this->image_infos) {
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

    for (ImageInfo im : image_infos) {
        if (frameID >= clicked_frame) {
            float minDist = 100;
            int minId = 0;
            int hbID = 0;
            bool target_found = false;
            cv::Point2f minPt;
            OpenPosePerson minHb;
            cv::Mat dum = im.image.clone();
//            cout << im.persons.size() << endl;
            for (OpenPosePerson hb: im.persons) {
                vector<cv::Point2f> bodyCoords = hb.getBodyCoord();
                if (target_found == false) {
                    for (int i = 0; i < bodyCoords.size(); i++) {
                        if ((i == 0) || (i == 14) || (i == 15) || (i == 16) || (i == 17)) {
                            float dist = yagi::calc2PointDistance(bodyCoords[i], prePt);
//                            cout << "dist = " << dist << endl;
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


                if (minDist != 100.0) {
                    image_infos[frameID].persons[minId].humanID = 1;
                    prePt = minPt;
                    preHb = minHb;
                    target_found = true;
                    for (cv::Point2f pt : im.persons[minId].getBodyCoord()) {
                        cv::circle(im.image, pt, 2, colors[3], 2);
                    }
//                    cv::imshow("target_person", im.image);
//                    cv::waitKey(1);
                }
            }

//            cv::imshow("targetRunner", im.image);
//            cv::waitKey(10);

            if (frameID == (image_infos.size() - 1))
                break;
        }
        frameID++;
    }
    cv::destroyAllWindows();
}

int FootPrint::clickLegPoint(){
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


    for(ImageInfo im: this->image_infos) {
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
        cout << "Legpoint" << this->image_infos[pt.second].persons[0].getBodyCoord()[10] << endl;
        cout << "Legpoint" << this->image_infos[pt.second].persons[0].getBodyCoord()[13] << endl;
    }

//    int frameID = 0;
//    cv::Point2f prePt = tracking_runner_point;
//    OpenPosePerson preHb;
//
//    for (ImageInfo im : image_infos) {
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
//                    image_infos[frameID].persons[minId].humanID = 1;
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
//            if (frameID == (image_infos.size() - 1))
//                break;
//        }
//        frameID++;
//    }
    cv::destroyAllWindows();
}


int FootPrint::findFootPrint(){
    int frame_num = 0;
    for(ImageInfo im: this->image_infos){
        if (frame_num % 10 == 0) {
            cout << frame_num << endl;
        }
        cv::Mat R_mat = cv::Mat::zeros(3,3,CV_32F);

        for(int i = 0; i < 3; i++){
            for(int j = 0; j < 3; j++){
                R_mat.at<float>(j, i) = im.camera._R[i * 3 + j];
            }
        }

        im.camera.R_inv = R_mat.inv();

        for(OpenPosePerson person: im.persons){
            if (person.humanID == 1){
                cv::Point2f leftLeg = person.getBodyCoord()[10];
                cv::Point2f rightLeg = person.getBodyCoord()[13];

                //関節分座標値を変更
//                leftLeg.y += 10;
//                rightLeg.y += 10;

//                //足領域が3次元点群のどこにあたるか
//                vector<int> leftLegAreaID = this->findFootPrintAreaIn3D(leftLeg, im.camera);
//                vector<int> rightLegAreaID = this->findFootPrintAreaIn3D(rightLeg, im.camera);
//
//                //足領域を赤く塗りつぶす
//                this->paintFootPrint(leftLegAreaID);
//                this->paintFootPrint(rightLegAreaID);

                //足領域が3次元点群のどこにあたるか
                set<int> leftLegAreaID = this->findFootPrintAreaIn3DSet(leftLeg, &im.camera, im.image);
                set<int> rightLegAreaID = this->findFootPrintAreaIn3DSet(rightLeg, &im.camera, im.image);

                //setリストに追加
//                for (int id: leftLegAreaID){
//                    this->leftFootPrintID.insert(id);
//                }
//                for (int id: rightLegAreaID){
//                    this->rightFootPrintID.insert(id);
//                }

                //vectorに投票
                if(frame_num > 0) {
                    for (int id: leftLegAreaID) {
                        this->left_vote[id] += 1;
                        LvoteRecord[id].frameNum.push_back(frame_num);
                    }
                    for (int id: rightLegAreaID) {
                        this->right_vote[id] += 1;
                        RvoteRecord[id].frameNum.push_back(frame_num);
                    }
                }
            }
        }
        frame_num++;
    }

    return 1;
}

int FootPrint::findClickedFootPrint(){
    int frame_num = 0;
    for (ImageInfo im: image_infos) {
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
    cv::Point3f vector_to_world;
    vector_to_world.x = (point.x - this->cameraParam[2]) / this->cameraParam[0];
    vector_to_world.y = (point.y - this->cameraParam[5]) / this->cameraParam[4];
    vector_to_world.z = 1.0;
    return vector_to_world;
}

cv::Point2f FootPrint::worldPointToImagePoint(cv::Point3f point, Camera* camera){


    //for project points
    //世界座標系からカメラ座標系への変換
    cv::Point3f camera_coord_point;
    vector<float> R = camera->_R;
    cv::Point3f T = camera->_T;

//    cv::Mat R_mat = cv::Mat::zeros(3,3,CV_64F);
//    for(int i = 0; i < 3; i++){
//        for(int j = 0; j < 3; j++){
//            R_mat.at<double>(j, i) = R[i * 3 + j];
//        }
//    }
////    cout << R_mat << endl;
//
//    cv::Mat T_mat = cv::Mat::zeros(3, 1, CV_64F);
//    T_mat.at<double>(0, 0) = -T.x;
//    T_mat.at<double>(1, 0) = -T.y;
//    T_mat.at<double>(2, 0) = -T.z;
////    cout << T_mat << endl;
//
//    cv::Mat A_mat = cv::Mat::zeros(3,3,CV_64F);
//    for(int i = 0; i < 3; i++){
//        for(int j = 0; j < 3; j++){
//            A_mat.at<double>(j, i) = cameraParam[i * 3 + j];
//        }
//    }
//
//    cv::Mat Coef = cv::Mat::zeros(4, 1, CV_64F);
//
//    vector<cv::Point3d> points;
//    vector<cv::Point2d> image_points;
//    points.push_back(point);
////    cout <<point <<  points[0] << endl;
//
//    cv::Mat r_vec;
//    cv::Rodrigues(R_mat.inv(), r_vec);
//    cv::projectPoints(points, R_mat.inv(), T_mat, A_mat, Coef, image_points);


//    camera_coord_point.x = R.at<float>(0, 0)*point.x + R.at<float>(0, 1)*point.y + R.at<float>(0, 2)*point.z;
//    camera_coord_point.y = R.at<float>(1, 0)*point.x + R.at<float>(1, 1)*point.y + R.at<float>(1, 2)*point.z;
//    camera_coord_point.z = R.at<float>(2, 0)*point.x + R.at<float>(2, 1)*point.y + R.at<float>(2, 2)*point.z;
//    camera_coord_point.x = R[0]*point.x + R[3]*point.y + R[6]*point.z;
//    camera_coord_point.y = R[1]*point.x + R[4]*point.y + R[7]*point.z;
//    camera_coord_point.z = R[2]*point.x + R[5]*point.y + R[8]*point.z;


    camera_coord_point = point;

//    camera_coord_point.x - T.x;
//    camera_coord_point.y - T.y;
//    camera_coord_point.z - T.z;
    point.x -= T.x;
    point.y -= T.y;
    point.z -= T.z;
    camera_coord_point.x = camera->R_inv.at<float>(0, 0)*point.x + camera->R_inv.at<float>(0, 1)*point.y + camera->R_inv.at<float>(0, 2)*point.z;
    camera_coord_point.y = camera->R_inv.at<float>(1, 0)*point.x + camera->R_inv.at<float>(1, 1)*point.y + camera->R_inv.at<float>(1, 2)*point.z;
    camera_coord_point.z = camera->R_inv.at<float>(2, 0)*point.x + camera->R_inv.at<float>(2, 1)*point.y + camera->R_inv.at<float>(2, 2)*point.z;

//    camera_coord_point.x = R[0]*point.x + R[1]*point.y + R[2]*point.z;
//    camera_coord_point.y = R[3]*point.x + R[4]*point.y + R[5]*point.z;
//    camera_coord_point.z = R[6]*point.x + R[7]*point.y + R[8]*point.z;

//    camera_coord_point.x - 8.91;
//    camera_coord_point.y - 0.33;
//    camera_coord_point.z + 4.49;

//    camera_coord_point = point;

    //カメラ座標系から正規化画像座標系への変換
    cv::Point2f normalized_point;
    normalized_point.x = camera_coord_point.x / camera_coord_point.z;
    normalized_point.y = camera_coord_point.y / camera_coord_point.z;

    //正規化画像座標系から画像座標系への変換
    cv::Point2f image_point;
//    cout << this->cameraParam[0] << " " << this->cameraParam[2] << endl;
//    cout << this->cameraParam[4] << " " << this->cameraParam[5] << endl;
//    image_point.x = (normalized_point.x * 800) + 960;
//    image_point.y = (normalized_point.y * 800) + 540;
    image_point.x = (normalized_point.x * this->cameraParam[0]) + this->cameraParam[2];
    image_point.y = (normalized_point.y * this->cameraParam[4]) + this->cameraParam[5];

//    cout << image_point << endl;
//    cout << image_points[0] << endl;

    return image_point;
//    return image_points[0];
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
    //plyfile出力
    cout << "[outputting ply file]:" << endl;

    //header
    string file_name;
    if (this->FACE_PLY)
        file_name= this->_projects_path + "/result/result_mesh.ply";
    else
        file_name= this->_projects_path + "/result/result_point.ply";

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