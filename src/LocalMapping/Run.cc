#include "Atlas.h"
#include "KeyFrame.h"
#include "LocalMapping.h"
#include "Optimizer.h"
#include "Tracking.h"

namespace ORB_SLAM3 {

void LocalMapping::Run() {
    mbFinished = false;

    while (1) {
        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(false);

        // Check if there are keyframes in the queue
        if (CheckNewKeyFrames() && !mbBadImu) {
            RunOnce();
        } else if (Stop() && !mbBadImu) {
            // Safe area to stop
            while (isStopped() && !CheckFinish()) {
                usleep(3000);
            }
            if (CheckFinish()) break;
        }

        ResetIfRequested();

        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(true);

        if (CheckFinish()) break;

        usleep(3000);
    }

    SetFinish();
}

float DistanceOfKeyFrames(KeyFrame& kf1, KeyFrame& kf2) {
    return (kf1.GetCameraCenter() - kf2.GetCameraCenter()).norm();
}

void LocalBA(const bool& mbInertial, KeyFrame* mpCurrentKeyFrame, float& mTinit,
             std::mutex& mMutexReset, bool& mbResetRequestedActiveMap,
             Map*& mpMapToReset, bool& mbBadImu, Tracking* mpTracker,
             const bool mbMonocular, bool& mbAbortBA) {
    // この変数群は、IMUが使えても使えなくてもBAに渡されるが、
    // IMUが使える場合には変更されない。
    // また、IMUが使えない場合には変更されるが、結果は使われない
    int num_FixedKF_BA = 0;
    int num_OptKF_BA = 0;
    int num_MPs_BA = 0;
    int num_edges_BA = 0;

    // IMUを用いており、実際にIMUが有効化されている場合はその情報を用いて
    // 動作を行う。
    if (mbInertial && mpCurrentKeyFrame->GetMap()->isImuInitialized()) {
        float dist = DistanceOfKeyFrames(*mpCurrentKeyFrame,
                                         *mpCurrentKeyFrame->mPrevKF) +
                     DistanceOfKeyFrames(*mpCurrentKeyFrame->mPrevKF,
                                         *mpCurrentKeyFrame->mPrevKF->mPrevKF);

        if (dist > 0.05)
            mTinit += mpCurrentKeyFrame->mTimeStamp -
                      mpCurrentKeyFrame->mPrevKF->mTimeStamp;
        if (!mpCurrentKeyFrame->GetMap()->GetIniertialBA2()) {
            if ((mTinit < 10.f) && (dist < 0.02)) {
                cout << "Not enough motion for initializing. "
                        "Reseting..."
                     << endl;
                unique_lock<mutex> lock(mMutexReset);
                mbResetRequestedActiveMap = true;
                mpMapToReset = mpCurrentKeyFrame->GetMap();
                mbBadImu = true;
            }
        }

        // 外れ値でない(Inlier)マップポイントの数が十分な場合にtrue
        bool bLarge = ((mpTracker->GetMatchesInliers() > 75) && mbMonocular) ||
                      ((mpTracker->GetMatchesInliers() > 100) && !mbMonocular);
        // IMUが使えるのでLocalInertialBA
        Optimizer::LocalInertialBA(
            mpCurrentKeyFrame, &mbAbortBA, mpCurrentKeyFrame->GetMap(),
            num_FixedKF_BA, num_OptKF_BA, num_MPs_BA, num_edges_BA, bLarge,
            !mpCurrentKeyFrame->GetMap()->GetIniertialBA2());
    } else {
        // IMUが使えないのでLocalBundleAdjustment
        Optimizer::LocalBundleAdjustment(
            mpCurrentKeyFrame, &mbAbortBA, mpCurrentKeyFrame->GetMap(),
            num_FixedKF_BA, num_OptKF_BA, num_MPs_BA, num_edges_BA);
    }
}

void LocalMapping::RunOnce() {
    // BoW conversion and insertion in Map
    ProcessNewKeyFrame();

    // Check recent MapPoints
    MapPointCulling();

    // Triangulate new MapPoints
    CreateNewMapPoints();

    mbAbortBA = false;

    // 新しいキーフレームが存在しないとき
    if (!CheckNewKeyFrames()) {
        // Find more matches in neighbor keyframes and fuse point
        // duplications 重複
        SearchInNeighbors();
    }

    if (CheckNewKeyFrames() || stopRequested()) {
        mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
        return;
    }

    // ある程度キーフレームが溜まっているならLocalBAをする
    if (mpAtlas->KeyFramesInMap() > 2) {
        LocalBA(mbInertial, mpCurrentKeyFrame, mTinit, mMutexReset,
                mbResetRequestedActiveMap, mpMapToReset, mbBadImu, mpTracker,
                mbMonocular, mbAbortBA);
    }

    // Initialize IMU here
    if (!mpCurrentKeyFrame->GetMap()->isImuInitialized() && mbInertial) {
        if (mbMonocular)
            InitializeIMU(1e2, 1e10, true);
        else
            InitializeIMU(1e2, 1e5, true);
    }

    // Check redundant local Keyframes
    KeyFrameCulling();

    if ((mTinit < 50.0f) && mbInertial) {
        if (mpCurrentKeyFrame->GetMap()->isImuInitialized() &&
            mpTracker->mState == Tracking::OK)  // Enter here everytime
                                                // local-mapping is called
        {
            if (!mpCurrentKeyFrame->GetMap()->GetIniertialBA1()) {
                if (mTinit > 5.0f) {
                    cout << "start VIBA 1" << endl;
                    mpCurrentKeyFrame->GetMap()->SetIniertialBA1();
                    if (mbMonocular)
                        InitializeIMU(1.f, 1e5, true);
                    else
                        InitializeIMU(1.f, 1e5, true);

                    cout << "end VIBA 1" << endl;
                }
            } else if (!mpCurrentKeyFrame->GetMap()->GetIniertialBA2()) {
                if (mTinit > 15.0f) {
                    cout << "start VIBA 2" << endl;
                    mpCurrentKeyFrame->GetMap()->SetIniertialBA2();
                    if (mbMonocular)
                        InitializeIMU(0.f, 0.f, true);
                    else
                        InitializeIMU(0.f, 0.f, true);

                    cout << "end VIBA 2" << endl;
                }
            }

            // scale refinement
            if (((mpAtlas->KeyFramesInMap()) <= 200) &&
                ((mTinit > 25.0f && mTinit < 25.5f) ||
                 (mTinit > 35.0f && mTinit < 35.5f) ||
                 (mTinit > 45.0f && mTinit < 45.5f) ||
                 (mTinit > 55.0f && mTinit < 55.5f) ||
                 (mTinit > 65.0f && mTinit < 65.5f) ||
                 (mTinit > 75.0f && mTinit < 75.5f))) {
                if (mbMonocular) ScaleRefinement();
            }
        }
    }

    mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
}

}  // namespace ORB_SLAM3
