cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

add_library(Openpose STATIC OpenPosePerson.cpp OpenPosePerson.h exportHumanPoseTxt.cpp myOpenPose.h)