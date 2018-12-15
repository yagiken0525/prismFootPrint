//
// Created by yagi on 18/07/18.
//

#include <opencv2/xfeatures2d.hpp>
#include "basicFunction.h"

using namespace std;
using namespace yagi;
using namespace cv;

//グローバル変数
struct mouseParam {
    int x;
    int y;
};
bool clicked = false;
cv::Point2f clicked_point;


//コールバック関数
void runnerCallBackFunc(int eventType, int x, int y, int flags, void *userdata) {
    switch (eventType) {
        case cv::EVENT_LBUTTONUP:
            std::cout << x << " , " << y << std::endl;
            clicked_point.x = x;
            clicked_point.y = y;
            clicked = true;
    }
}


void yagi::clickPoints(cv::Mat image, vector<cv::Point2f> & clickedPoints, const string file_name) {
    vector<cv::Scalar> colors;
    setColor(&colors);
    ofstream outputTxt(file_name);

    //最初のフレームでスタートラインクリック
    mouseParam mouseEvent;
    string windowName = "click points (Q: finish clicking)";
    cv::namedWindow(windowName, CV_WINDOW_AUTOSIZE);
    cv::setMouseCallback(windowName, runnerCallBackFunc, &mouseEvent);
    
    while (1) {
        cv::imshow(windowName, image);
        int key = cv::waitKey(1);

        if (clicked) {
            clicked = false;
            cv::circle(image, clicked_point, 2, colors[0], 2);
            cv::Point2f pt(clicked_point.x, clicked_point.y);
            clickedPoints.push_back(pt);
            outputTxt << pt.x << " " << pt.y << endl;
        }
        if (key == 'q')
            break;
    }

    cv::destroyAllWindows();
}
