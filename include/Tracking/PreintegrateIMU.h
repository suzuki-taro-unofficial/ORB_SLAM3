#ifndef ORB_SLAM3_TRACKING_PREINTEGRATEIMU_H_
#define ORB_SLAM3_TRACKING_PREINTEGRATEIMU_H_

#include "Frame.h"
#include "ImuTypes.h"

namespace ORB_SLAM3 {
namespace tracking {

/// フレーム/キーフレーム間のIMUの事前積分値を管理する
class PreintegrateIMU {
public:
    PreintegrateIMU(IMU::Bias bias, IMU::Calib &calib);

    /// `lastFrame`と`currFrame`の間にあるIMUデータ列`vImuFromLastFrame`を事前積分して
    /// `mpImuPreintegratedFromLastFrame`と`mpImuPreintegratedFromLastKF`を更新する。
    void Preintegrate(const std::vector<IMU::Point> &vImuFromLastFrame,
                      const Frame &lastFrame, const Frame &currFrame);

    /// `mpImuPreintegratedFromLastKF`を初期化する。
    ///
    /// @param bias      事前積分に用いるIMUのバイアス
    /// @param pCalib    事前積分に用いるIMUのキャリブレーション
    /// @param deleteOld trueでリセット前の事前積分値をdeleteする
    void InitPreintegrationFromLastKF(IMU::Bias bias, IMU::Calib &calib,
                                      bool deleteOld);

    /// 前回のフレームからの事前積分値
    IMU::Preintegrated *mpImuPreintegratedFromLastFrame;

    /// 前回のキーフレームからの事前積分値
    IMU::Preintegrated *mpImuPreintegratedFromLastKF;
};

}  // namespace tracking
}  // namespace ORB_SLAM3

#endif  // ORB_SLAM3_TRACKING_PREINTEGRATEIMU_H_
