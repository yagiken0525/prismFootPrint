//
// Created by yagi on 18/10/07.
//

#include "k_means.h"

using namespace std;
using namespace cv;

void k_means::clustering(std::vector<cv::Point3f> points, int k) {
    int point_size = points.size();
    vector<vector<float>> class_points;
    int pt_num = 0;
    for(cv::Point3f pt: points){
        vector<float> class_pt(0, 4);
        class_pt[0] = pt.x;
        class_pt[1] = pt.y;
        class_pt[2] = pt.z;
        class_pt[3] = (pt_num % k);
        class_points.push_back(class_pt);
    }

    
}