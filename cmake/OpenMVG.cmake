#${project name}config.cmake を探す場所を指定
#set(OpenMVG_DIR /home/yagi/UserLibrary/opencv-2.4.13/Build)

#findcmake,configcmakeを探す
find_package(OpenMVG REQUIRED)

if (OpenMVG_FOUND)
    #include一覧に格納
    include_directories(${OpenMVG_INCLUDE_DIRS})
    #linkライブラリ一覧に格納
    set(OpenMVG_LIBS
            OpenMVG::openMVG_camera
            OpenMVG::openMVG_exif
            OpenMVG::openMVG_features
            OpenMVG::openMVG_geodesy
            OpenMVG::openMVG_geometry
            OpenMVG::openMVG_graph
            OpenMVG::openMVG_image
            OpenMVG::openMVG_linearProgramming
            OpenMVG::openMVG_matching
            OpenMVG::openMVG_matching_image_collection
            OpenMVG::openMVG_multiview
            OpenMVG::openMVG_numeric
            OpenMVG::openMVG_robust_estimation
            OpenMVG::openMVG_sfm
            OpenMVG::openMVG_system)
    link_directories (${OpenMVG_LIBS})
    #debug用出力
    message(STATUS "OpenMVG_DIR: ${OpenMVG_DIR}")
#    message(STATUS "OpenMVG version: ${OpenMVG_VERSION}")
#    message(STATUS "OpenMVG include: ${OpenMVG_INCLUDE_DIRS}")
    message(STATUS "OpenMVG libraries : ${OpenMVG_LIBS}")
else()
    message(WARNING "Could not find OpenMVG.")
endif()