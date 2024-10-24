add_library(${PROJECT_NAME} SHARED
    System.cc
    Tracking.cc
    LocalMapping.cc
    LoopClosing.cc
    ORBextractor.cc
    ORBmatcher.cc
    FrameDrawer.cc
    Converter.cc
    MapPoint.cc
    KeyFrame.cc
    Atlas.cc
    Map.cc
    MapDrawer.cc
    Optimizer.cc
    Frame.cc
    KeyFrameDatabase.cc
    Sim3Solver.cc
    Viewer.cc
    ImuTypes.cc
    G2oTypes.cc
    CameraModels/Pinhole.cpp
    CameraModels/KannalaBrandt8.cpp
    OptimizableTypes.cpp
    MLPnPsolver.cpp
    GeometricTools.cc
    TwoViewReconstruction.cc
    Config.cc
    Settings.cc
    ${PROJECT_SOURCE_DIR}/include/System.h
    ${PROJECT_SOURCE_DIR}/include/Tracking.h
    ${PROJECT_SOURCE_DIR}/include/LocalMapping.h
    ${PROJECT_SOURCE_DIR}/include/LoopClosing.h
    ${PROJECT_SOURCE_DIR}/include/ORBextractor.h
    ${PROJECT_SOURCE_DIR}/include/ORBmatcher.h
    ${PROJECT_SOURCE_DIR}/include/FrameDrawer.h
    ${PROJECT_SOURCE_DIR}/include/Converter.h
    ${PROJECT_SOURCE_DIR}/include/MapPoint.h
    ${PROJECT_SOURCE_DIR}/include/KeyFrame.h
    ${PROJECT_SOURCE_DIR}/include/Atlas.h
    ${PROJECT_SOURCE_DIR}/include/Map.h
    ${PROJECT_SOURCE_DIR}/include/MapDrawer.h
    ${PROJECT_SOURCE_DIR}/include/Optimizer.h
    ${PROJECT_SOURCE_DIR}/include/Frame.h
    ${PROJECT_SOURCE_DIR}/include/KeyFrameDatabase.h
    ${PROJECT_SOURCE_DIR}/include/Sim3Solver.h
    ${PROJECT_SOURCE_DIR}/include/Viewer.h
    ${PROJECT_SOURCE_DIR}/include/ImuTypes.h
    ${PROJECT_SOURCE_DIR}/include/G2oTypes.h
    ${PROJECT_SOURCE_DIR}/include/CameraModels/GeometricCamera.h
    ${PROJECT_SOURCE_DIR}/include/CameraModels/Pinhole.h
    ${PROJECT_SOURCE_DIR}/include/CameraModels/KannalaBrandt8.h
    ${PROJECT_SOURCE_DIR}/include/OptimizableTypes.h
    ${PROJECT_SOURCE_DIR}/include/MLPnPsolver.h
    ${PROJECT_SOURCE_DIR}/include/GeometricTools.h
    ${PROJECT_SOURCE_DIR}/include/TwoViewReconstruction.h
    ${PROJECT_SOURCE_DIR}/include/SerializationUtils.h
    ${PROJECT_SOURCE_DIR}/include/Config.h
    ${PROJECT_SOURCE_DIR}/include/Settings.h
)

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
    -lboost_serialization
    -lcrypto
)

target_include_directories(${PROJECT_NAME}
    PUBLIC ${PROJECT_SOURCE_DIR}/include
    PUBLIC ${PROJECT_SOURCE_DIR}/include/CameraModels
    PUBLIC ${PROJECT_SOURCE_DIR}
    PUBLIC ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
    PRIVATE ${EIGEN3_INCLUDE_DIR}
    PRIVATE ${Pangolin_INCLUDE_DIRS}
)

# If RealSense SDK is found the library is added and its examples compiled
if(realsense2_FOUND)
    target_include_directories(${PROJECT_NAME}
        PRIVATE ${realsense_INCLUDE_DIR}
    )
    target_link_libraries(${PROJECT_NAME}
        ${realsense2_LIBRARY}
    )
endif()
