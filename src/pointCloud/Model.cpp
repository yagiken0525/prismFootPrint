//
// Created by yagi on 18/07/24.
//

#include "Model.h"
#include <fstream>
#include "../basicFunction/basicFunction.h"


using namespace std;

bool Model::readModel(string filename){
    ifstream ifs(filename);
    string buf;
    unsigned int i;

    if(!ifs.is_open()){
        std::cout << "cannot open "<< filename << std::endl;
        return false;
    }
    cout << "[.ply file correctly loaded]:" << endl;
    while(ifs >> buf){

        if(buf =="vertex"){
            ifs >> vertices_num;
            if(ifs.fail()){
                std::cout <<"error! vertices_num is not int"<<std::endl;
            }
            cout << "vertex num: " << vertices_num << endl;
        }
        if(buf =="face"){
            ifs >> faces_num;
            if(ifs.fail()){
                std::cout <<"error! faces_num is not int"<<std::endl;
            }
            cout << "face num: " << faces_num << endl;
        }
        if(buf == "end_header")
            break;
    }
    if(vertices_num==0 && faces_num==0){
        return false;
    }
    std::getline(ifs,buf);

    vector<string> vertex;
    vector<string> face;
    for(i= 0;i<vertices_num;i++){
        std::getline(ifs,buf);
        vertex = yagi::split(buf, ' ');
        cv::Point3f pt(stof(vertex[0]), stof(vertex[1]), stof(vertex[2]));
        this->vertices.push_back(pt);
    }
    faces.resize(faces_num);
    for(i= 0;i<faces_num;i++){
        std::getline(ifs,buf);
        face = yagi::split(buf, ' ');
        faces[i].verticies.push_back(stoi(face[1]));
        faces[i].verticies.push_back(stoi(face[2]));
        faces[i].verticies.push_back(stoi(face[3]));
    }

    if(vertices.size() == vertices_num && faces.size() == faces_num){
        return true;
    }else{
        return false;
    }
}

bool Model::readVertixModel(string filename){
    ifstream ifs(filename);
    string buf;
    unsigned int i;

    if(!ifs.is_open()){
        std::cout << "cannot open "<< filename << std::endl;
        return false;
    }
    cout << "[.ply file correctly loaded]:" << endl;
    while(ifs >> buf){

        if(buf =="vertex"){
            ifs >> vertices_num;
            if(ifs.fail()){
                std::cout <<"error! vertices_num is not int"<<std::endl;
            }
            cout << "vertex num: " << vertices_num << endl;
        }
        if(buf =="face"){
            ifs >> faces_num;
            if(ifs.fail()){
                std::cout <<"error! faces_num is not int"<<std::endl;
            }
            cout << "face num: " << faces_num << endl;
        }
        if(buf == "end_header")
            break;
    }
    if(vertices_num==0 ){
        return false;
    }
    std::getline(ifs,buf);
    this->vertices.resize(vertices_num);

    vector<string> vertex;
    vector<string> face;
    for(i= 0;i<vertices_num;i++){
        std::getline(ifs,buf);
        vertex = yagi::split(buf, ' ');
        this->vertices[i].x = stof(vertex[0]);
        this->vertices[i].y = stof(vertex[1]);
        this->vertices[i].z = stof(vertex[2]);

    }
//    faces.resize(faces_num);
//    for(i= 0;i<faces_num;i++){
//        std::getline(ifs,buf);
//        face = yagi::split(buf, ' ');
//        faces[i].verticies.push_back(stoi(face[1]));
//        faces[i].verticies.push_back(stoi(face[2]));
//        faces[i].verticies.push_back(stoi(face[3]));
//    }
//
//    if(vertices.size() == vertices_num && faces.size() == faces_num){
//        return true;
//    }else{
//        return false;
//    }
}


bool Model::readTranslationModel(string filename){
    ifstream ifs(filename);
    string buf;
    unsigned int i;

    if(!ifs.is_open()){
        std::cout << "cannot open "<< filename << std::endl;
        return false;
    }
    cout << "[.ply file correctly loaded]:" << endl;
    while(ifs >> buf){

        if(buf =="vertex"){
            ifs >> vertices_num;
            if(ifs.fail()){
                std::cout <<"error! vertices_num is not int"<<std::endl;
            }
            cout << "vertex num: " << vertices_num << endl;
        }
        if(buf =="face"){
            ifs >> faces_num;
            if(ifs.fail()){
                std::cout <<"error! faces_num is not int"<<std::endl;
            }
            cout << "face num: " << faces_num << endl;
        }
        if(buf == "end_header")
            break;
    }
    if(vertices_num==0 || faces_num==0){
        return false;
    }
    std::getline(ifs,buf);
    this->vertices.resize(vertices_num);

    vector<string> vertex;
    vector<string> face;
    for(i= 0;i<vertices_num;i++){
        std::getline(ifs,buf);
        vertex = yagi::split(buf, ' ');
        this->vertices[i].x = stof(vertex[0]);
        this->vertices[i].y = stof(vertex[1]);
        this->vertices[i].z = stof(vertex[2]);

    }
    faces.resize(faces_num);
    for(i= 0;i<faces_num;i++){
        std::getline(ifs,buf);
        face = yagi::split(buf, ' ');
        faces[i].verticies.push_back(stoi(face[1]));
        faces[i].verticies.push_back(stoi(face[2]));
        faces[i].verticies.push_back(stoi(face[3]));
    }

    if(vertices.size() == vertices_num && faces.size() == faces_num){
        return true;
    }else{
        return false;
    }
}


bool Model::readModelWithColor(string filename, cv::Scalar color){
    ifstream ifs(filename);
    string buf;
    unsigned int i;

    if(!ifs.is_open()){
        std::cout << "cannot open "<< filename << std::endl;
        return false;
    }
    cout << "[.ply file correctly loaded]:" << endl;
    while(ifs >> buf){

        if(buf =="vertex"){
            ifs >> vertices_num;
            if(ifs.fail()){
                std::cout <<"error! vertices_num is not int"<<std::endl;
            }
            cout << "vertex num: " << vertices_num << endl;
        }
        if(buf =="face"){
            ifs >> faces_num;
            if(ifs.fail()){
                std::cout <<"error! faces_num is not int"<<std::endl;
            }
            cout << "face num: " << faces_num << endl;
        }
        if(buf == "end_header")
            break;
    }
    if(vertices_num==0){
        return false;
    }
    std::getline(ifs,buf);
//    this->vertices.resize(vertices_num);

    vector<string> vertex;
    vector<string> face;
    for(i= 0;i<vertices_num;i++){
        std::getline(ifs,buf);
        vertex = yagi::split(buf, ' ');
        cv::Scalar pointColor(stof(vertex[3]), stof(vertex[4]), stof(vertex[5]));
        if(pointColor == color) {
            //cout << i << endl;
            cv::Point3f new_point(stof(vertex[0]),stof(vertex[1]),stof(vertex[2]));
            this->vertices.push_back(new_point);
        }

    }

    cout <<"vertix size " << this->vertices.size() << endl;
    if(vertices.size() == vertices_num && faces.size() == faces_num){
        return true;
    }else{
        return false;
    }
}

void Model::loadFrom4DcvMat(cv::Mat pointsMat) {
    for (int i = 0; i < pointsMat.cols; i++) {
        cv::Point3f pt(pointsMat.at<float>(i, 0),
                       pointsMat.at<float>(i, 1),
                       pointsMat.at<float>(i, 2)
        );
        this->vertices.push_back(pt);
    }
}


void Model::savePly(string saveFilePath){
    //plyfile出力
    cout << "[outputting ply file]:" << endl;

    //header
    string file_name;
    file_name = saveFilePath;

    ofstream outputfile(file_name);
    outputfile << "ply" << endl;
    outputfile << "format ascii 1.0" << endl;
    outputfile << "comment VCGLIB generated" << endl;
    outputfile << "element vertex " + to_string(this->vertices.size()) << endl;
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

    for (int i = 0; i < this->vertices.size(); i++){

        outputfile  << this->vertices[i].x << " "
                    << this->vertices[i].y << " "
                    << this->vertices[i].z << " "
                    << 255 << " "
                    << 255 << " "
                    << 255 << " "
                    << 255 << endl;


    }

    outputfile.close();
    cout << "ply correctly exported" << endl;
}