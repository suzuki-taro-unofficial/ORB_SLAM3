add_library(${PROJECT_NAME} SHARED
    System.cc
    Tracking.cc
    LocalMapping.cc
    LocalMapping/CreateNewMapPoints.cc
    LocalMapping/KeyFrameCulling.cc
    LocalMapping/InitializeIMU.cc
    LocalMapping/SearchInNeighbors.cc
    LocalMapping/Run.cc
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
    CameraModels/Pinhole.cc
    CameraModels/KannalaBrandt8.cc
    OptimizableTypes.cc
    MLPnPsolver.cc
    GeometricTools.cc
    TwoViewReconstruction.cc
    Config.cc
    Settings.cc
)

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    DBoW2
    g2o
    -lboost_serialization
    -lcrypto
)

target_include_directories(${PROJECT_NAME}
    PUBLIC ${PROJECT_SOURCE_DIR}/include
    PUBLIC ${PROJECT_SOURCE_DIR}/Thirdparty/g2o
    PUBLIC ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2
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
