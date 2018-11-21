//
// Created by yagi on 18/10/17.
//
// ----------------------------- OpenPose C++ API Tutorial - Example 1 - Body from image -----------------------------
// It reads an image, process it, and displays it with the pose keypoints.

// Command-line user intraface
#define OPENPOSE_FLAGS_DISABLE_POSE

#include "myOpenPose.h"
#include "../../src/basicFunction/basicFunction.h"
#include <sys/stat.h>
#include<iostream>
#include<fstream>

using namespace cv;
using namespace std;


// Custom OpenPose flags
// Producer
//DEFINE_string(image_path, "examples/media/COCO_val2014_000000000192.jpg", "Process an image. Read all standard formats (jpg, png, bmp, etc.).");

// This worker will just read and return all the jpg files in a directory
void display(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr)
{
    // User's displaying/saving/other processing here
    // datum.cvOutputData: rendered frame with pose or heatmaps
    // datum.poseKeypoints: Array<float> with the estimated pose
    if (datumsPtr != nullptr && !datumsPtr->empty())
    {
        // Display image
        cv::imshow("User worker GUI", datumsPtr->at(0).cvOutputData);
        cv::waitKey(1);
    }
    else
        op::log("Nullptr or empty datumsPtr found.", op::Priority::High);
}

void printKeypoints(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr)
{
    // Example: How to use the pose keypoints
    if (datumsPtr != nullptr && !datumsPtr->empty())
    {
        // Alternative 1
//        op::log("Body keypoints: " + datumsPtr->at(0).poseKeypoints.toString());

        // // Alternative 2
//        op::log(datumsPtr->at(0).poseKeypoints);

        // // Alternative 3
        cout << datumsPtr->at(0).poseKeypoints.getVolume() << endl;
        std::cout << datumsPtr->at(0).poseKeypoints << std::endl;

        // // Alternative 4 - Accesing each element of the keypoints
        // op::log("\nKeypoints:");
        // const auto& poseKeypoints = datumsPtr->at(0).poseKeypoints;
        // op::log("Person pose keypoints:");
        // for (auto person = 0 ; person < poseKeypoints.getSize(0) ; person++)
        // {
        //     op::log("Person " + std::to_string(person) + " (x, y, score):");
        //     for (auto bodyPart = 0 ; bodyPart < poseKeypoints.getSize(1) ; bodyPart++)
        //     {
        //         std::string valueToPrint;
        //         for (auto xyscore = 0 ; xyscore < poseKeypoints.getSize(2) ; xyscore++)
        //             valueToPrint += std::to_string(   poseKeypoints[{person, bodyPart, xyscore}]   ) + " ";
        //         op::log(valueToPrint);
        //     }
        // }
        // op::log(" ");
    }
    else
        op::log("Nullptr or empty datumsPtr found.", op::Priority::High);
}

void yagi::outputTextFromImage(const std::string video_path, const std::string output_path, vector<cv::Mat> images){

    //ディレクトリ作成
    const char *cstr = output_path.c_str();
    if (mkdir(cstr, 0777) == 0) {
        printf("directory correctly generated\n");
    } else {
        printf("directory already exists\n");
    }
    string output_image_path = output_path + "/images/";
    const char *cstr2 = output_image_path.c_str();
    if (mkdir(cstr2, 0777) == 0) {
        printf("directory correctly generated\n");
    } else {
        printf("directory already exists\n");
    }

    ofstream outputfile(output_path + "/human_pose_info.txt");
    ofstream imageoutputfile(output_path + "/imagelist.txt");

    op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};
    opWrapper.start();

    // Process and display image

    for(int i=0; i<images.size();i++){

//        cv::resize(images[i],images[i],Size(), 640.0/images[i].cols, 320.0/images[i].rows);

        //cv::Mat dummy = cv::Mat::zeros(images[i].cols, images[i].rows * (images[i].), CV_32FC3);

        auto datumProcessed = opWrapper.emplaceAndPop(images[i]);
        if (datumProcessed != nullptr)
        {
//            printKeypoints(datumProcessed);
            if (datumProcessed != nullptr && !datumProcessed->empty())
            {
                // // Alternative 3
//                std::cout << i << std::endl;
//                std::cout << "a " << datumProcessed->at(0).poseKeypoints.getNumberDimensions() << std::endl;
//                std::cout << "b " << datumProcessed->at(0).poseKeypoints.getVolume() << std::endl;
//                std::cout << "c " << datumProcessed->at(0).poseKeypoints.getSize().size()<< std::endl;
//                std::cout << "d " << datumProcessed->at(0).poseKeypoints << std::endl;
//                outputfile << "Frame: " << i << endl;
                int elem_num = datumProcessed->at(0).poseKeypoints.getVolume();
                for (int j = 0; j < elem_num; j++){
                    if(j % 75 == 0){
                        outputfile << "Person " << (j/75) << " (x, y, score):" << endl;
                    }
                    outputfile << datumProcessed->at(0).poseKeypoints[j] << " ";
                    if(j % 3 == 2){
                        outputfile << endl;
                    }
                }

            }
            else
                op::log("Nullptr or empty datumsPtr found.", op::Priority::High);

            display(datumProcessed);
            cv::imwrite(output_image_path + yagi::digitString(i, 4) + ".jpg", datumProcessed->at(0).cvOutputData);
            imageoutputfile << output_image_path + yagi::digitString(i,4) + ".jpg" << endl;
        }
        else
            op::log("Image could not be processed.", op::Priority::High);

    }

    outputfile.close();
    // Return successful message
    op::log("Stopping OpenPose...", op::Priority::High);
}



void yagi::outputTextFromVideo(const std::string video_path, const std::string output_path){

    //ディレクトリ作成
    const char *cstr = output_path.c_str();
    if (mkdir(cstr, 0777) == 0) {
        printf("directory correctly generated\n");
    } else {
        printf("directory already exists\n");
    }
    string output_image_path = output_path + "/images/";
    const char *cstr2 = output_image_path.c_str();
    if (mkdir(cstr2, 0777) == 0) {
        printf("directory correctly generated\n");
    } else {
        printf("directory already exists\n");
    }

    ofstream outputfile(output_path + "/human_pose_info.txt");
    ofstream imageoutputfile(output_path + "/imagelist.txt");

    Mat img;
    VideoCapture cap(video_path);
    op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};
    opWrapper.start();

    // Process and display image
    int max_frame=cap.get(CV_CAP_PROP_FRAME_COUNT);
    for(int i=0; i<max_frame;i++){
        cap>>img ; //1フレーム分取り出してimgに保持させる
        cv::resize(img,img,Size(), 640.0/img.cols, 320.0/img.rows);

        //cv::Mat dummy = cv::Mat::zeros(img.cols, img.rows * (img.), CV_32FC3);

        auto datumProcessed = opWrapper.emplaceAndPop(img);
        if (datumProcessed != nullptr)
        {
//            printKeypoints(datumProcessed);
            if (datumProcessed != nullptr && !datumProcessed->empty())
            {
                // // Alternative 3
//                std::cout << i << std::endl;
//                std::cout << "a " << datumProcessed->at(0).poseKeypoints.getNumberDimensions() << std::endl;
//                std::cout << "b " << datumProcessed->at(0).poseKeypoints.getVolume() << std::endl;
//                std::cout << "c " << datumProcessed->at(0).poseKeypoints.getSize().size()<< std::endl;
//                std::cout << "d " << datumProcessed->at(0).poseKeypoints << std::endl;
//                outputfile << "Frame: " << i << endl;
                int elem_num = datumProcessed->at(0).poseKeypoints.getVolume();
                for (int j = 0; j < elem_num; j++){
                    if(j % 75 == 0){
                        outputfile << "Person " << (j/75) << " (x, y, score):" << endl;
                    }
                    outputfile << datumProcessed->at(0).poseKeypoints[j] << " ";
                    if(j % 3 == 2){
                        outputfile << endl;
                    }
                }

            }
            else
                op::log("Nullptr or empty datumsPtr found.", op::Priority::High);

            display(datumProcessed);
            cv::imwrite(output_image_path + yagi::digitString(i, 4) + ".jpg", datumProcessed->at(0).cvOutputData);
            imageoutputfile << output_image_path + yagi::digitString(i,4) + ".jpg" << endl;
        }
        else
            op::log("Image could not be processed.", op::Priority::High);

    }

    outputfile.close();
    // Return successful message
    op::log("Stopping OpenPose...", op::Priority::High);
}

