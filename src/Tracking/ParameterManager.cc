#include "Tracking/ParameterManager.h"

#include "Atlas.h"
#include "FrameDrawer.h"
#include "Settings.h"

namespace ORB_SLAM3 {
namespace tracking {

ParameterManager::ParameterManager(int sensor, Atlas *pAtlas,
                                   FrameDrawer *pFrameDrawer,
                                   Settings *pSettings) {
    mpCamera = pSettings->camera1();
    mpCamera = pAtlas->AddCamera(mpCamera);

    if (pSettings->needToUndistort()) {
        mDistCoef = pSettings->camera1DistortionCoef();
    } else {
        mDistCoef = cv::Mat::zeros(4, 1, CV_32F);
    }

    // TODO: missing image scaling and rectification
    mImageScale = 1.0f;

    mK = cv::Mat::eye(3, 3, CV_32F);
    mK.at<float>(0, 0) = mpCamera->getParameter(0);
    mK.at<float>(1, 1) = mpCamera->getParameter(1);
    mK.at<float>(0, 2) = mpCamera->getParameter(2);
    mK.at<float>(1, 2) = mpCamera->getParameter(3);

    mK_.setIdentity();
    mK_(0, 0) = mpCamera->getParameter(0);
    mK_(1, 1) = mpCamera->getParameter(1);
    mK_(0, 2) = mpCamera->getParameter(2);
    mK_(1, 2) = mpCamera->getParameter(3);

    if ((sensor == System::STEREO || sensor == System::IMU_STEREO ||
         sensor == System::IMU_RGBD) &&
        pSettings->cameraType() == Settings::KannalaBrandt) {
        mpCamera2 = pSettings->camera2();
        mpCamera2 = pAtlas->AddCamera(mpCamera2);

        mTlr = pSettings->Tlr();

        pFrameDrawer->both = true;
    }

    if (sensor == System::STEREO || sensor == System::RGBD ||
        sensor == System::IMU_STEREO || sensor == System::IMU_RGBD) {
        mbf = pSettings->bf();
        mThDepth = pSettings->b() * pSettings->thDepth();
    }

    if (sensor == System::RGBD || sensor == System::IMU_RGBD) {
        mDepthMapFactor = pSettings->depthMapFactor();
        if (fabs(mDepthMapFactor) < 1e-5)
            mDepthMapFactor = 1;
        else
            mDepthMapFactor = 1.0f / mDepthMapFactor;
    }

    mMinFrames = 0;
    mMaxFrames = pSettings->fps();
    mbRGB = pSettings->rgb();

    // ORB parameters
    int nFeatures = pSettings->nFeatures();
    int nLevels = pSettings->nLevels();
    int fIniThFAST = pSettings->initThFAST();
    int fMinThFAST = pSettings->minThFAST();
    float fScaleFactor = pSettings->scaleFactor();

    mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels,
                                          fIniThFAST, fMinThFAST);

    if (sensor == System::STEREO || sensor == System::IMU_STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels,
                                               fIniThFAST, fMinThFAST);

    if (sensor == System::MONOCULAR || sensor == System::IMU_MONOCULAR)
        mpIniORBextractor = new ORBextractor(5 * nFeatures, fScaleFactor,
                                             nLevels, fIniThFAST, fMinThFAST);

    // IMU parameters
    Sophus::SE3f Tbc = pSettings->Tbc();
    mInsertKFsLost = pSettings->insertKFsWhenLost();
    mImuFreq = pSettings->imuFrequency();
    mImuPer = 0.001;  // 1.0 / (double) mImuFreq;     //TODO: ESTO ESTA BIEN?
    float Ng = pSettings->noiseGyro();
    float Na = pSettings->noiseAcc();
    float Ngw = pSettings->gyroWalk();
    float Naw = pSettings->accWalk();

    const float sf = sqrt(mImuFreq);
    mpImuCalib = new IMU::Calib(Tbc, Ng * sf, Na * sf, Ngw / sf, Naw / sf);
}

}  // namespace tracking
}  // namespace ORB_SLAM3
