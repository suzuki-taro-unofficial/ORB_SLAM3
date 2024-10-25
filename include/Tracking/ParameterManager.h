#ifndef ORB_SLAM3_TRACKING_PARAMETERMANAGER_H_
#define ORB_SLAM3_TRACKING_PARAMETERMANAGER_H_

#include "GeometricCamera.h"
#include "ImuTypes.h"
#include "ORBextractor.h"

namespace ORB_SLAM3 {

class FrameDrawer;
class Atlas;
class Settings;

namespace tracking {

struct ParameterManager {
    ParameterManager(int sensor, Atlas *pAtlas, FrameDrawer *pFrameDrawer,
                     Settings *pSettings);

    // Imu calibration parameters
    IMU::Calib *mpImuCalib;

    // ORB
    ORBextractor *mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor *mpIniORBextractor;

    GeometricCamera *mpCamera, *mpCamera2;

    // Calibration matrix
    cv::Mat mK;
    Eigen::Matrix3f mK_;
    cv::Mat mDistCoef;
    float mbf;
    float mImageScale;

    float mImuFreq;
    double mImuPer;
    bool mInsertKFsLost;

    // New KeyFrame rules (according to fps)
    int mMinFrames, mMaxFrames;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two
    // keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values
    // are scaled.
    float mDepthMapFactor;

    // Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    Sophus::SE3f mTlr;
    bool mFastInit;
};

}  // namespace tracking
}  // namespace ORB_SLAM3

#endif  // ORB_SLAM3_TRACKING_PARAMETERMANAGER_H_
