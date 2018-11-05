//
// Created by yagi on 18/07/24.
//

#include "Camera.h"
#include "../basicFunction/basicFunction.h"

using namespace std;


//int Camera::loadCameraRt(std::string file_name){
//    ifstream ifs(file_name);
//    string buf;
//
//    if(!ifs.is_open()){
//        std::cout << "cannot open "<< file_name << std::endl;
//        return false;
//    }
//
//    int line_num = 0;
//    while(ifs >> buf){
//        std::getline(ifs,buf);
//        if ((line_num % 14) == 5){
//            vector<string> translation_tmp  = yagi::split(buf, ' ');
//            this->t.push_back(stof(translation_tmp[0]));
//            this->t.push_back(stof(translation_tmp[1]));
//            this->t.push_back(stof(translation_tmp[2]));
//        }else if(((line_num % 14) >= 9) && ((line_num % 14) <= 11)) {
//            vector<string> rotation_tmp  = yagi::split(buf, ' ');
//            this->R.push_back(stof(rotation_tmp[0]));
//            this->R.push_back(stof(rotation_tmp[1]));
//            this->R.push_back(stof(rotation_tmp[2]));
//        }else if((line_num % 14) == 0){
//
//        }
//        line_num++;
//    }
//    return 1;
//}