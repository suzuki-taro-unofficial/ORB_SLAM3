#include "LocalMapping.h"
#include "Optimizer.h"

namespace ORB_SLAM3 {
/**
 * InitializeIMUから切り出し
 * KFをprevKFで追っていって、古いものを先頭にした結果を返す
 * prevKFはMapの切り替わりで途切れる
 */
vector<KeyFrame*> RetrieveAllKFinTemporalOrder(KeyFrame* kf) {
    list<KeyFrame*> lpKF;
    while (kf->mPrevKF) {
        lpKF.push_front(kf);
        kf = kf->mPrevKF;
    }
    lpKF.push_front(kf);
    vector<KeyFrame*> vpKF(lpKF.begin(), lpKF.end());

    return vpKF;
}

/// @brief IMUの初期化し、最適化も施す
/// @param priorG
/// @param priorA
/// @param bFIBA Full IMU Bundle Adjustment を行うかどうか(true -> 行う)
void LocalMapping::InitializeIMU(float priorG, float priorA, bool bFIBA) {
    if (mbResetRequested) return;

    const float minTime = mbMonocular ? 2.0 : 1.0;
    const int nMinKF = 10;

    if (mpAtlas->KeyFramesInMap() < nMinKF) return;

    // Retrieve all keyframe in temporal order
    vector<KeyFrame*> vpKF = RetrieveAllKFinTemporalOrder(mpCurrentKeyFrame);

    // KFが少なすぎるか、経過時間が短すぎるなら処理をしない
    if (vpKF.size() < nMinKF) return;
    // SystemやTrackingが使う
    // Mapで一番古いKFの時間が入る。
    mFirstTs = vpKF.front()->mTimeStamp;
    if (mpCurrentKeyFrame->mTimeStamp - mFirstTs < minTime) return;

    bInitializing = true;

    while (CheckNewKeyFrames()) {
        ProcessNewKeyFrame();
        vpKF.push_back(mpCurrentKeyFrame);
    }

    const int N = vpKF.size();
    IMU::Bias b(0, 0, 0, 0, 0, 0);  // unused

    // Compute and KF velocities mRwg estimation
    if (!mpCurrentKeyFrame->GetMap()->isImuInitialized()) {
        Eigen::Matrix3f Rwg;
        Eigen::Vector3f dirG;
        dirG.setZero();
        for (vector<KeyFrame*>::iterator itKF = vpKF.begin();
             itKF != vpKF.end(); itKF++) {
            if (!(*itKF)->mpImuPreintegrated) continue;
            if (!(*itKF)->mPrevKF) continue;

            dirG -= (*itKF)->mPrevKF->GetImuRotation() *
                    (*itKF)->mpImuPreintegrated->GetUpdatedDeltaVelocity();
            Eigen::Vector3f _vel = ((*itKF)->GetImuPosition() -
                                    (*itKF)->mPrevKF->GetImuPosition()) /
                                   (*itKF)->mpImuPreintegrated->dT;
            (*itKF)->SetVelocity(_vel);
            (*itKF)->mPrevKF->SetVelocity(_vel);
        }

        dirG = dirG / dirG.norm();
        Eigen::Vector3f gI(0.0f, 0.0f, -1.0f);
        Eigen::Vector3f v = gI.cross(dirG);
        const float nv = v.norm();
        const float cosg = gI.dot(dirG);
        const float ang = acos(cosg);
        Eigen::Vector3f vzg = v * ang / nv;
        Rwg = Sophus::SO3f::exp(vzg).matrix();
        mRwg = Rwg.cast<double>();
        mTinit = mpCurrentKeyFrame->mTimeStamp - mFirstTs;
    } else {
        mRwg = Eigen::Matrix3d::Identity();
        mbg = mpCurrentKeyFrame->GetGyroBias().cast<double>();
        mba = mpCurrentKeyFrame->GetAccBias().cast<double>();
    }

    mScale = 1.0;

    mInitTime = mpTracker->mLastFrame.mTimeStamp - vpKF.front()->mTimeStamp;

    Optimizer::InertialOptimization(mpAtlas->GetCurrentMap(), mRwg, mScale, mbg,
                                    mba, mbMonocular, infoInertial, false,
                                    false, priorG, priorA);

    if (mScale < 1e-1) {
        cout << "scale too small" << endl;
        bInitializing = false;
        return;
    }

    // Before this line we are not changing the map
    {
        unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);
        if ((fabs(mScale - 1.f) > 0.00001) || !mbMonocular) {
            Sophus::SE3f Twg(mRwg.cast<float>().transpose(),
                             Eigen::Vector3f::Zero());
            mpAtlas->GetCurrentMap()->ApplyScaledRotation(Twg, mScale, true);
            mpTracker->UpdateFrameIMU(mScale, vpKF[0]->GetImuBias(),
                                      mpCurrentKeyFrame);
        }

        // Check if initialization OK
        if (!mpAtlas->isImuInitialized())
            for (int i = 0; i < N; i++) {
                KeyFrame* pKF2 = vpKF[i];
                pKF2->bImu = true;
            }
    }

    mpTracker->UpdateFrameIMU(1.0, vpKF[0]->GetImuBias(), mpCurrentKeyFrame);
    if (!mpAtlas->isImuInitialized()) {
        mpAtlas->SetImuInitialized();
        mpTracker->t0IMU = mpTracker->mCurrentFrame.mTimeStamp;
        mpCurrentKeyFrame->bImu = true;
    }

    if (bFIBA) {
        if (priorA != 0.f)
            Optimizer::FullInertialBA(mpAtlas->GetCurrentMap(), 100, false,
                                      mpCurrentKeyFrame->mnId, NULL, true,
                                      priorG, priorA);
        else
            Optimizer::FullInertialBA(mpAtlas->GetCurrentMap(), 100, false,
                                      mpCurrentKeyFrame->mnId, NULL, false);
    }

    Verbose::PrintMess("Global Bundle Adjustment finished\nUpdating map ...",
                       Verbose::VERBOSITY_NORMAL);

    // Get Map Mutex
    unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);

    unsigned long GBAid = mpCurrentKeyFrame->mnId;

    // Process keyframes in the queue
    while (CheckNewKeyFrames()) {
        ProcessNewKeyFrame();
        vpKF.push_back(mpCurrentKeyFrame);
    }

    // Correct keyframes starting at map first keyframe
    list<KeyFrame*> lpKFtoCheck(
        mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.begin(),
        mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.end());

    while (!lpKFtoCheck.empty()) {
        KeyFrame* pKF = lpKFtoCheck.front();
        const set<KeyFrame*> sChilds = pKF->GetChilds();
        Sophus::SE3f Twc = pKF->GetPoseInverse();
        for (set<KeyFrame*>::const_iterator sit = sChilds.begin();
             sit != sChilds.end(); sit++) {
            KeyFrame* pChild = *sit;
            if (!pChild || pChild->isBad()) continue;

            if (pChild->mnBAGlobalForKF != GBAid) {
                Sophus::SE3f Tchildc = pChild->GetPose() * Twc;
                pChild->mTcwGBA = Tchildc * pKF->mTcwGBA;

                Sophus::SO3f Rcor =
                    pChild->mTcwGBA.so3().inverse() * pChild->GetPose().so3();
                if (pChild->isVelocitySet()) {
                    pChild->mVwbGBA = Rcor * pChild->GetVelocity();
                } else {
                    Verbose::PrintMess("Child velocity empty!! ",
                                       Verbose::VERBOSITY_NORMAL);
                }

                pChild->mBiasGBA = pChild->GetImuBias();
                pChild->mnBAGlobalForKF = GBAid;
            }
            lpKFtoCheck.push_back(pChild);
        }

        pKF->mTcwBefGBA = pKF->GetPose();
        pKF->SetPose(pKF->mTcwGBA);

        if (pKF->bImu) {
            pKF->mVwbBefGBA = pKF->GetVelocity();
            pKF->SetVelocity(pKF->mVwbGBA);
            pKF->SetNewBias(pKF->mBiasGBA);
        } else {
            cout << "KF " << pKF->mnId << " not set to inertial!! \n";
        }

        lpKFtoCheck.pop_front();
    }

    // Correct MapPoints
    const vector<MapPoint*> vpMPs = mpAtlas->GetCurrentMap()->GetAllMapPoints();

    for (size_t i = 0; i < vpMPs.size(); i++) {
        MapPoint* pMP = vpMPs[i];

        if (pMP->isBad()) continue;

        if (pMP->mnBAGlobalForKF == GBAid) {
            // If optimized by Global BA, just update
            pMP->SetWorldPos(pMP->mPosGBA);
        } else {
            // Update according to the correction of its reference keyframe
            KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();

            if (pRefKF->mnBAGlobalForKF != GBAid) continue;

            // Map to non-corrected camera
            Eigen::Vector3f Xc = pRefKF->mTcwBefGBA * pMP->GetWorldPos();

            // Backproject using corrected camera
            pMP->SetWorldPos(pRefKF->GetPoseInverse() * Xc);
        }
    }

    Verbose::PrintMess("Map updated!", Verbose::VERBOSITY_NORMAL);

    mnKFs = vpKF.size();
    mIdxInit++;

    for (list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(),
                                   lend = mlNewKeyFrames.end();
         lit != lend; lit++) {
        (*lit)->SetBadFlag();
        delete *lit;
    }
    mlNewKeyFrames.clear();

    mpTracker->mState = Tracking::OK;
    bInitializing = false;

    mpCurrentKeyFrame->GetMap()->IncreaseChangeIndex();

    return;
}

}  // namespace ORB_SLAM3
