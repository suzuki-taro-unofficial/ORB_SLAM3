#include "Tracking/PreintegrateIMU.h"

namespace ORB_SLAM3 {
namespace tracking {

PreintegrateIMU::PreintegrateIMU(IMU::Bias bias, IMU::Calib &calib)
    : mpImuPreintegratedFromLastFrame(nullptr),
      mpImuPreintegratedFromLastKF(nullptr) {
    mpImuPreintegratedFromLastKF = new IMU::Preintegrated(bias, calib);
}

void PreintegrateIMU::Preintegrate(
    const std::vector<IMU::Point> &vImuFromLastFrame, const Frame &lastFrame,
    const Frame &currFrame) {
    // From Tracking::PreintegrateIMU

    mpImuPreintegratedFromLastFrame =
        new IMU::Preintegrated(lastFrame.mImuBias, currFrame.mImuCalib);

    const int n = vImuFromLastFrame.size();
    for (int i = 0; i < n - 1; i++) {
        float tstep;
        Eigen::Vector3f acc, angVel;
        if ((i == 0) && (i < (n - 1))) {
            float tab = vImuFromLastFrame[i + 1].t - vImuFromLastFrame[i].t;
            float tini =
                vImuFromLastFrame[i].t - currFrame.mpPrevFrame->mTimeStamp;
            acc = (vImuFromLastFrame[i].a + vImuFromLastFrame[i + 1].a -
                   (vImuFromLastFrame[i + 1].a - vImuFromLastFrame[i].a) *
                       (tini / tab)) *
                  0.5f;
            angVel = (vImuFromLastFrame[i].w + vImuFromLastFrame[i + 1].w -
                      (vImuFromLastFrame[i + 1].w - vImuFromLastFrame[i].w) *
                          (tini / tab)) *
                     0.5f;
            tstep =
                vImuFromLastFrame[i + 1].t - currFrame.mpPrevFrame->mTimeStamp;
        } else if (i < (n - 1)) {
            acc = (vImuFromLastFrame[i].a + vImuFromLastFrame[i + 1].a) * 0.5f;
            angVel =
                (vImuFromLastFrame[i].w + vImuFromLastFrame[i + 1].w) * 0.5f;
            tstep = vImuFromLastFrame[i + 1].t - vImuFromLastFrame[i].t;
        } else if ((i > 0) && (i == (n - 1))) {
            float tab = vImuFromLastFrame[i + 1].t - vImuFromLastFrame[i].t;
            float tend = vImuFromLastFrame[i + 1].t - currFrame.mTimeStamp;
            acc = (vImuFromLastFrame[i].a + vImuFromLastFrame[i + 1].a -
                   (vImuFromLastFrame[i + 1].a - vImuFromLastFrame[i].a) *
                       (tend / tab)) *
                  0.5f;
            angVel = (vImuFromLastFrame[i].w + vImuFromLastFrame[i + 1].w -
                      (vImuFromLastFrame[i + 1].w - vImuFromLastFrame[i].w) *
                          (tend / tab)) *
                     0.5f;
            tstep = currFrame.mTimeStamp - vImuFromLastFrame[i].t;
        } else if ((i == 0) && (i == (n - 1))) {
            acc = vImuFromLastFrame[i].a;
            angVel = vImuFromLastFrame[i].w;
            tstep = currFrame.mTimeStamp - currFrame.mpPrevFrame->mTimeStamp;
        }

        mpImuPreintegratedFromLastKF->IntegrateNewMeasurement(acc, angVel,
                                                              tstep);
        mpImuPreintegratedFromLastFrame->IntegrateNewMeasurement(acc, angVel,
                                                                 tstep);
    }
}

void PreintegrateIMU::InitPreintegrationFromLastKF(IMU::Bias bias,
                                                   IMU::Calib &calib,
                                                   bool deleteOld) {
    if (deleteOld) {
        delete mpImuPreintegratedFromLastKF;
    }
    mpImuPreintegratedFromLastKF = new IMU::Preintegrated(bias, calib);
}

}  // namespace tracking
}  // namespace ORB_SLAM3
