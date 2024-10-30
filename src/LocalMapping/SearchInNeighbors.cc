#include "KeyFrame.h"
#include "LocalMapping.h"
#include "ORBmatcher.h"

namespace ORB_SLAM3 {
std::vector<ORB_SLAM3::KeyFrame*> GetNeighbors(const bool& mbMonocular,
                                               KeyFrame* mpCurrentKeyFrame,
                                               const bool& mbAbortBA,
                                               const bool& mbInertial);

void LocalMapping::SearchInNeighbors() {
    vector<KeyFrame*> vpTargetKFs =
        GetNeighbors(mbMonocular, mpCurrentKeyFrame, mbAbortBA, mbInertial);

    // Search matches by projection from current KF in target KFs
    ORBmatcher matcher;
    vector<MapPoint*> vpMapPointMatches =
        mpCurrentKeyFrame->GetMapPointMatches();
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
            if (pMP->isBad() ||
                pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
                continue;
            pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
            vpFuseCandidates.push_back(pMP);
        }
    }

    matcher.Fuse(mpCurrentKeyFrame, vpFuseCandidates);
    if (mpCurrentKeyFrame->NLeft != -1)
        matcher.Fuse(mpCurrentKeyFrame, vpFuseCandidates, true);

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

void getFromCovisible(vector<KeyFrame*>& vpTargetKFs, const int nn,
                      KeyFrame* mpCurrentKeyFrame);
void getFromCovisibleOfCovisible(vector<KeyFrame*>& vpTargetKFs,
                                 KeyFrame* mpCurrentKeyFrame,
                                 const bool& mbAbortBA);
void getFromPrevKF(vector<KeyFrame*>& vpTargetKFs, KeyFrame* mpCurrentKeyFrame);

/**
 * CurrentKeyFrameからCovisibilityGraphでつながっているKFなどを取得する。
 *
 * SearchInNeighborsから切り出した
 */
std::vector<ORB_SLAM3::KeyFrame*> GetNeighbors(const bool& mbMonocular,
                                               KeyFrame* mpCurrentKeyFrame,
                                               const bool& mbAbortBA,
                                               const bool& mbInertial) {
    // Retrieve neighbor keyframes
    int nn = 10;
    if (mbMonocular) nn = 30;

    vector<KeyFrame*> vpTargetKFs;

    getFromCovisible(vpTargetKFs, nn, mpCurrentKeyFrame);
    getFromCovisibleOfCovisible(vpTargetKFs, mpCurrentKeyFrame, mbAbortBA);
    if (mbInertial) {
        getFromPrevKF(vpTargetKFs, mpCurrentKeyFrame);
    }

    return vpTargetKFs;
}

void getFromCovisible(vector<KeyFrame*>& vpTargetKFs, const int nn,
                      KeyFrame* mpCurrentKeyFrame) {
    const vector<KeyFrame*> vpNeighKFs =
        mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
    for (auto pKFi : vpNeighKFs) {
        if (pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
            continue;
        vpTargetKFs.push_back(pKFi);
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;
    }
}

void getFromCovisibleOfCovisible(vector<KeyFrame*>& vpTargetKFs,
                                 KeyFrame* mpCurrentKeyFrame,
                                 const bool& mbAbortBA) {
    // Add some covisible of covisible
    // Extend to some second neighbors if abort is not requested
    for (int i = 0, imax = vpTargetKFs.size(); i < imax; i++) {
        const vector<KeyFrame*> vpSecondNeighKFs =
            vpTargetKFs[i]->GetBestCovisibilityKeyFrames(20);
        for (auto pKFi2 : vpSecondNeighKFs) {
            if (pKFi2->isBad() ||
                pKFi2->mnFuseTargetForKF == mpCurrentKeyFrame->mnId ||
                pKFi2->mnId == mpCurrentKeyFrame->mnId)
                continue;
            vpTargetKFs.push_back(pKFi2);
            pKFi2->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;
        }
        if (mbAbortBA) break;
    }
}

void getFromPrevKF(vector<KeyFrame*>& vpTargetKFs,
                   KeyFrame* mpCurrentKeyFrame) {
    // Extend to temporal neighbors
    KeyFrame* pKFi = mpCurrentKeyFrame->mPrevKF;
    while (vpTargetKFs.size() < 20 && pKFi) {
        if (pKFi->isBad() ||
            pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId) {
            pKFi = pKFi->mPrevKF;
            continue;
        }
        vpTargetKFs.push_back(pKFi);
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;
        pKFi = pKFi->mPrevKF;
    }
}

}  // namespace ORB_SLAM3
