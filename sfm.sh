#!/usr/bin/zsh

echo structure-from-motion
echo input project name
read projectName
echo $projectName

sfmDR_home=~/sfmDR
openMVG=~/work/openMVG_Build
openMVG_bin=~/work/openMVG_Build/Linux-x86_64-RELEASE
projects=/home/yagi/CLionProjects/prismFootPrint/Data/Projects

openMVG_project=~/work/openMVG_Build/projects
openMVS_bin=~/UserLibrary/openMVS_build/bin
cLionProject=/home/yagi/CLionProjects/sfmDR_footPrint/Data/Projects

echo $openMVG
echo $openMVG_bin
echo $openMVG_project
echo $openMVS_bin

echo openMVG_main_SfMInit_ImageListing
$openMVG_bin/openMVG_main_SfMInit_ImageListing -i $projects/$projectName/sfm/sfmImages/ -o $projects/$projectName/sfm/sfmImages/ -k "1958.09372;0;987.72;0;1964.5410;504.71;0;0;1"

echo openMVG_main_ComputeFeatures
$openMVG_bin/openMVG_main_ComputeFeatures -i $projects/$projectName/sfm/sfmImages/sfm_data.json -o $projects/$projectName/sfm/matches/ -p HIGH -f 1

echo openMVG_main_ComputeMatches
$openMVG_bin/openMVG_main_ComputeMatches -i $projects/$projectName/sfm/sfmImages/sfm_data.json -o $projects/$projectName/sfm/matches/ -f 1

echo openMVG_main_IncrementalSfM
$openMVG_bin/openMVG_main_IncrementalSfM -i $projects/$projectName/sfm/sfmImages/sfm_data.json -m $projects/$projectName/sfm/matches/ -o $projects/$projectName/sfm/results/

echo openMVG_main_openMVG2NVM
$openMVG_bin/openMVG_main_openMVG2NVM -i $projects/$projectName/sfm/results/sfm_data.bin -o $sfmDR_projects/$projectName/

echo openMVG_main_ConvertSfM_DataFormat
$openMVG_bin/openMVG_main_ConvertSfM_DataFormat -i $projects/$projectName/sfm/results/sfm_data.bin -o $projects/$projectName/sfm/cameraRt.json -E



