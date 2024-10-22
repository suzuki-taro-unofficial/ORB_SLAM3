/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#include "LocalMapping.h"

#include <mutex>

#include "GeometricTools.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"

namespace ORB_SLAM3 {

LocalMapping::LocalMapping(System* pSys, Atlas* pAtlas, const float bMonocular, bool bInertial,
                           const string& _strSeqName)
    : mpSystem(pSys),
      mbMonocular(bMonocular),
      mbInertial(bInertial),  // inertial 慣性
      mbResetRequested(false),
      mbResetRequestedActiveMap(false),
      mbFinishRequested(false),
      mbFinished(true),
      mpAtlas(pAtlas),
      bInitializing(false),
      mbAbortBA(false),
      mbStopped(false),
      mbStopRequested(false),
      mbNotStop(false),
      mbAcceptKeyFrames(true),
      mIdxInit(0),
      mScale(1.0),
      mInitSect(0),
      mbNotBA1(true),
      mbNotBA2(true),
      mIdxIteration(0),
      infoInertial(Eigen::MatrixXd::Zero(9, 9)) {
    mnMatchesInliers = 0;

    mbBadImu = false;

    mTinit = 0.f;

    mNumLM = 0;
    mNumKFCulling = 0;
}

void LocalMapping::SetLoopCloser(LoopClosing* pLoopCloser) { mpLoopCloser = pLoopCloser; }

void LocalMapping::SetTracker(Tracking* pTracker) { mpTracker = pTracker; }

float LocalMapping::DistanceOfKeyFrames(KeyFrame& kf1, KeyFrame& kf2) {
    return (kf1.GetCameraCenter() - kf2.GetCameraCenter()).norm();
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

    int num_FixedKF_BA = 0;
    int num_OptKF_BA = 0;
    int num_MPs_BA = 0;
    int num_edges_BA = 0;

    if (!CheckNewKeyFrames() && !stopRequested()) {
        // ある程度キーフレームが溜まっているならLocalBAをする
        if (mpAtlas->KeyFramesInMap() > 2) {
            // IMUを用いており、実際にIMUが有効化されている場合はその情報を用いて
            // 動作を行う。
            if (mbInertial && mpCurrentKeyFrame->GetMap()->isImuInitialized()) {
                float dist = DistanceOfKeyFrames(*mpCurrentKeyFrame, *mpCurrentKeyFrame->mPrevKF) +
                             DistanceOfKeyFrames(*mpCurrentKeyFrame->mPrevKF, *mpCurrentKeyFrame->mPrevKF->mPrevKF);

                if (dist > 0.05) mTinit += mpCurrentKeyFrame->mTimeStamp - mpCurrentKeyFrame->mPrevKF->mTimeStamp;
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
                Optimizer::LocalInertialBA(mpCurrentKeyFrame, &mbAbortBA, mpCurrentKeyFrame->GetMap(), num_FixedKF_BA,
                                           num_OptKF_BA, num_MPs_BA, num_edges_BA, bLarge,
                                           !mpCurrentKeyFrame->GetMap()->GetIniertialBA2());
            } else {
                // IMUが使えないのでLocalBundleAdjustment
                Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame, &mbAbortBA, mpCurrentKeyFrame->GetMap(),
                                                 num_FixedKF_BA, num_OptKF_BA, num_MPs_BA, num_edges_BA);
            }
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
                mpTracker->mState == Tracking::OK)  // Enter here everytime local-mapping
                                                    // is called
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
                    ((mTinit > 25.0f && mTinit < 25.5f) || (mTinit > 35.0f && mTinit < 35.5f) ||
                     (mTinit > 45.0f && mTinit < 45.5f) || (mTinit > 55.0f && mTinit < 55.5f) ||
                     (mTinit > 65.0f && mTinit < 65.5f) || (mTinit > 75.0f && mTinit < 75.5f))) {
                    if (mbMonocular) ScaleRefinement();
                }
            }
        }
    }

    mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
}

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

void LocalMapping::InsertKeyFrame(KeyFrame* pKF) {
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    mbAbortBA = true;
}

bool LocalMapping::CheckNewKeyFrames() {
    unique_lock<mutex> lock(mMutexNewKFs);
    return (!mlNewKeyFrames.empty());
}

void LocalMapping::ProcessNewKeyFrame() {
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
    }

    // Compute Bags of Words structures
    mpCurrentKeyFrame->ComputeBoW();

    // Associate MapPoints to the new keyframe and update normal and descriptor
    const vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

    for (size_t i = 0; i < vpMapPointMatches.size(); i++) {
        MapPoint* pMP = vpMapPointMatches[i];
        if (pMP) {
            if (!pMP->isBad()) {
                if (!pMP->IsInKeyFrame(mpCurrentKeyFrame)) {
                    pMP->AddObservation(mpCurrentKeyFrame, i);
                    pMP->UpdateNormalAndDepth();
                    pMP->ComputeDistinctiveDescriptors();
                } else  // this can only happen for new stereo points inserted
                        // by the Tracking
                {
                    mlpRecentAddedMapPoints.push_back(pMP);
                }
            }
        }
    }

    // Update links in the Covisibility Graph
    mpCurrentKeyFrame->UpdateConnections();

    // Insert Keyframe in Map
    mpAtlas->AddKeyFrame(mpCurrentKeyFrame);
}

void LocalMapping::EmptyQueue() {
    while (CheckNewKeyFrames()) ProcessNewKeyFrame();
}

/**
 * 最近追加されたマップポイントを精査し、無効なポイントや観測が少ないポイントを削除する
 */
void LocalMapping::MapPointCulling() {
    // Check Recent Added MapPoints
    list<MapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

    int nThObs;
    if (mbMonocular)
        nThObs = 2;
    else
        nThObs = 3;
    const int cnThObs = nThObs;

    int borrar = mlpRecentAddedMapPoints.size();

    while (lit != mlpRecentAddedMapPoints.end()) {
        MapPoint* pMP = *lit;

        if (pMP->isBad())
            lit = mlpRecentAddedMapPoints.erase(lit);
        else if (pMP->GetFoundRatio() < 0.25f) {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        } else if (((int)nCurrentKFid - (int)pMP->mnFirstKFid) >= 2 && pMP->Observations() <= cnThObs) {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        } else if (((int)nCurrentKFid - (int)pMP->mnFirstKFid) >= 3)
            lit = mlpRecentAddedMapPoints.erase(lit);
        else {
            lit++;
            borrar--;
        }
    }
}

/**
 * CurrentKeyFrameからCovisibilityGraphでつながっているKFなどを取得する。
 *
 * SearchInNeighborsから切り出した
 */
std::vector<KeyFrame*> LocalMapping::GetNeighbors() {
    // Retrieve neighbor keyframes
    int nn = 10;
    if (mbMonocular) nn = 30;
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
    vector<KeyFrame*> vpTargetKFs;
    for (auto pKFi : vpNeighKFs) {
        if (pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId) continue;
        vpTargetKFs.push_back(pKFi);
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;
    }

    // Add some covisible of covisible
    // Extend to some second neighbors if abort is not requested
    for (int i = 0, imax = vpTargetKFs.size(); i < imax; i++) {
        const vector<KeyFrame*> vpSecondNeighKFs = vpTargetKFs[i]->GetBestCovisibilityKeyFrames(20);
        for (auto pKFi2 : vpSecondNeighKFs) {
            if (pKFi2->isBad() || pKFi2->mnFuseTargetForKF == mpCurrentKeyFrame->mnId ||
                pKFi2->mnId == mpCurrentKeyFrame->mnId)
                continue;
            vpTargetKFs.push_back(pKFi2);
            pKFi2->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;
        }
        if (mbAbortBA) break;
    }

    // Extend to temporal neighbors
    if (mbInertial) {
        KeyFrame* pKFi = mpCurrentKeyFrame->mPrevKF;
        while (vpTargetKFs.size() < 20 && pKFi) {
            if (pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId) {
                pKFi = pKFi->mPrevKF;
                continue;
            }
            vpTargetKFs.push_back(pKFi);
            pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;
            pKFi = pKFi->mPrevKF;
        }
    }

    return vpTargetKFs;
}

void LocalMapping::SearchInNeighbors() {
    vector<KeyFrame*> vpTargetKFs = GetNeighbors();

    // Search matches by projection from current KF in target KFs
    ORBmatcher matcher;
    vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for (auto pKFi : vpTargetKFs) {
        matcher.Fuse(pKFi, vpMapPointMatches);
        /// matcher.Fuseの第三引数はfloat型なので普通にバグ
        if (pKFi->NLeft != -1) matcher.Fuse(pKFi, vpMapPointMatches, true);
    }

    if (mbAbortBA) return;

    // Search matches by projection from target KFs in current KF
    vector<MapPoint*> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size() * vpMapPointMatches.size());

    for (auto pKFi : vpTargetKFs) {
        vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();

        for (auto pMP : vpMapPointsKFi) {
            if (!pMP) continue;
            if (pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId) continue;
            pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
            vpFuseCandidates.push_back(pMP);
        }
    }

    matcher.Fuse(mpCurrentKeyFrame, vpFuseCandidates);
    if (mpCurrentKeyFrame->NLeft != -1) matcher.Fuse(mpCurrentKeyFrame, vpFuseCandidates, true);

    // Update points
    vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for (size_t i = 0, iend = vpMapPointMatches.size(); i < iend; i++) {
        MapPoint* pMP = vpMapPointMatches[i];
        if (pMP && !pMP->isBad()) {
            pMP->ComputeDistinctiveDescriptors();
            pMP->UpdateNormalAndDepth();
        }
    }

    // Update connections in covisibility graph
    mpCurrentKeyFrame->UpdateConnections();
}

void LocalMapping::RequestStop() {
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
    unique_lock<mutex> lock2(mMutexNewKFs);
    mbAbortBA = true;
}

bool LocalMapping::Stop() {
    unique_lock<mutex> lock(mMutexStop);
    if (mbStopRequested && !mbNotStop) {
        mbStopped = true;
        cout << "Local Mapping STOP" << endl;
        return true;
    }

    return false;
}

bool LocalMapping::isStopped() {
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool LocalMapping::stopRequested() {
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

void LocalMapping::Release() {
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if (mbFinished) return;
    mbStopped = false;
    mbStopRequested = false;
    for (list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend = mlNewKeyFrames.end(); lit != lend; lit++)
        delete *lit;
    mlNewKeyFrames.clear();

    cout << "Local Mapping RELEASE" << endl;
}

bool LocalMapping::AcceptKeyFrames() {
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void LocalMapping::SetAcceptKeyFrames(bool flag) {
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames = flag;
}

bool LocalMapping::SetNotStop(bool flag) {
    unique_lock<mutex> lock(mMutexStop);

    if (flag && mbStopped) return false;

    mbNotStop = flag;

    return true;
}

void LocalMapping::InterruptBA() { mbAbortBA = true; }

void LocalMapping::RequestReset() {
    {
        unique_lock<mutex> lock(mMutexReset);
        cout << "LM: Map reset recieved" << endl;
        mbResetRequested = true;
    }
    cout << "LM: Map reset, waiting..." << endl;

    while (1) {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if (!mbResetRequested) break;
        }
        usleep(3000);
    }
    cout << "LM: Map reset, Done!!!" << endl;
}

void LocalMapping::RequestResetActiveMap(Map* pMap) {
    {
        unique_lock<mutex> lock(mMutexReset);
        cout << "LM: Active map reset recieved" << endl;
        mbResetRequestedActiveMap = true;
        mpMapToReset = pMap;
    }
    cout << "LM: Active map reset, waiting..." << endl;

    while (1) {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if (!mbResetRequestedActiveMap) break;
        }
        usleep(3000);
    }
    cout << "LM: Active map reset, Done!!!" << endl;
}

void LocalMapping::ResetIfRequested() {
    bool executed_reset = false;
    {
        unique_lock<mutex> lock(mMutexReset);
        if (mbResetRequested) {
            executed_reset = true;

            cout << "LM: Reseting Atlas in Local Mapping..." << endl;
            mlNewKeyFrames.clear();
            mlpRecentAddedMapPoints.clear();
            mbResetRequested = false;
            mbResetRequestedActiveMap = false;

            // Inertial parameters
            mTinit = 0.f;
            mbNotBA2 = true;
            mbNotBA1 = true;
            mbBadImu = false;

            mIdxInit = 0;

            cout << "LM: End reseting Local Mapping..." << endl;
        }

        if (mbResetRequestedActiveMap) {
            executed_reset = true;
            cout << "LM: Reseting current map in Local Mapping..." << endl;
            mlNewKeyFrames.clear();
            mlpRecentAddedMapPoints.clear();

            // Inertial parameters
            mTinit = 0.f;
            mbNotBA2 = true;
            mbNotBA1 = true;
            mbBadImu = false;

            mbResetRequested = false;
            mbResetRequestedActiveMap = false;
            cout << "LM: End reseting Local Mapping..." << endl;
        }
    }
    if (executed_reset) cout << "LM: Reset free the mutex" << endl;
}

void LocalMapping::RequestFinish() {
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LocalMapping::CheckFinish() {
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LocalMapping::SetFinish() {
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool LocalMapping::isFinished() {
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

/**
 * - 新しいキーフレームがあれば先に前処理(`ProcessNewKeyFrame`)を行う
 * - IMU最適化？(`InertialOptimization`)をする
 */
void LocalMapping::ScaleRefinement() {
    // Minimum number of keyframes to compute a solution
    // Minimum time (seconds) between first and last keyframe to compute a
    // solution. Make the difference between monocular and stereo
    // unique_lock<mutex> lock0(mMutexImuInit);
    if (mbResetRequested) return;

    while (CheckNewKeyFrames()) {
        ProcessNewKeyFrame();
    }

    mRwg = Eigen::Matrix3d::Identity();
    mScale = 1.0;

    Optimizer::InertialOptimization(mpAtlas->GetCurrentMap(), mRwg, mScale);

    if (mScale < 1e-1)  // 1e-1
    {
        cout << "scale too small" << endl;
        bInitializing = false;
        return;
    }

    Sophus::SO3d so3wg(mRwg);
    // Before this line we are not changing the map
    unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);
    if ((fabs(mScale - 1.f) > 0.002) || !mbMonocular) {
        Sophus::SE3f Tgw(mRwg.cast<float>().transpose(), Eigen::Vector3f::Zero());
        mpAtlas->GetCurrentMap()->ApplyScaledRotation(Tgw, mScale, true);
        mpTracker->UpdateFrameIMU(mScale, mpCurrentKeyFrame->GetImuBias(), mpCurrentKeyFrame);
    }

    for (list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend = mlNewKeyFrames.end(); lit != lend; lit++) {
        (*lit)->SetBadFlag();
        delete *lit;
    }
    mlNewKeyFrames.clear();

    // To perform pose-inertial opt w.r.t. last keyframe
    mpCurrentKeyFrame->GetMap()->IncreaseChangeIndex();

    return;
}

bool LocalMapping::IsInitializing() { return bInitializing; }

double LocalMapping::GetCurrKFTime() {
    if (mpCurrentKeyFrame) {
        return mpCurrentKeyFrame->mTimeStamp;
    } else
        return 0.0;
}

KeyFrame* LocalMapping::GetCurrKF() { return mpCurrentKeyFrame; }

}  // namespace ORB_SLAM3
