//
// Created by yagi on 18/07/24.
//

#ifndef SFMDR_FOOTPRINT_MODEL_H
#define SFMDR_FOOTPRINT_MODEL_H

#include <iostream>
#include <vector>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

class Model {
public:
    ~Model(){vertices.clear();
            faces.clear();
            vertices_num = 0;
            faces_num = 0;};
    bool readModel(std::string filename);
    bool readModelWithColor(std::string filename, cv::Scalar color);
    bool readTranslationModel(std::string filename);
    bool readVertixModel(std::string filename);
    void loadFrom4DcvMat(cv::Mat points);
//    void loadFrom3DpointsVector(std::vector<cv::Point3f> points);
    void savePly(std::string saveFilePath);

    struct Face{
        std::vector<int> verticies;
    };

    int vertices_num;
    int faces_num;

    std::vector<cv::Point3f> vertices;
    std::vector<Face> faces;

private:


};


#endif //SFMDR_FOOTPRINT_MODEL_H
