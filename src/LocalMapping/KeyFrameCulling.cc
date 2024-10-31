#include "Atlas.h"
#include "KeyFrame.h"
#include "LocalMapping.h"

namespace ORB_SLAM3 {

/// 冗長なキーフレームのチェック（ローカルキーフレームのみ）
/// そのキーフレームが見ているマップポイントの90%が、少なくとも他の3つのキーフレーム（同じか、より細かいスケール）で見ている場合、そのキーフレームは冗長とみなされます。
void LocalMapping::KeyFrameCulling() {
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees,
    // are seen in at least other 3 keyframes (in the same or finer scale) We
    // only consider close stereo points
    const int Nd = 21;
    mpCurrentKeyFrame->UpdateBestCovisibles();
    vector<KeyFrame*> vpLocalKeyFrames =
        mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    float redundant_th;
    if (!mbInertial)
        redundant_th = 0.9;
    else if (mbMonocular)
        redundant_th = 0.9;
    else
        redundant_th = 0.5;

    const bool bInitImu = mpAtlas->isImuInitialized();
    int count = 0;

    // Compoute last KF from optimizable window:
    unsigned int last_ID;
    if (mbInertial) {
        int count = 0;
        KeyFrame* aux_KF = mpCurrentKeyFrame;
        while (count < Nd && aux_KF->mPrevKF) {
            aux_KF = aux_KF->mPrevKF;
            count++;
        }
        last_ID = aux_KF->mnId;
    }

    for (vector<KeyFrame*>::iterator vit = vpLocalKeyFrames.begin(),
                                     vend = vpLocalKeyFrames.end();
         vit != vend; vit++) {
        count++;
        KeyFrame* pKF = *vit;

        if ((pKF->mnId == pKF->GetMap()->GetInitKFid()) || pKF->isBad())
            continue;
        const vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

        int nObs = 3;
        const int thObs = nObs;
        int nRedundantObservations = 0;
        int nMPs = 0;
        for (size_t i = 0, iend = vpMapPoints.size(); i < iend; i++) {
            MapPoint* pMP = vpMapPoints[i];
            if (pMP) {
                if (!pMP->isBad()) {
                    if (!mbMonocular) {
                        if (pKF->mvDepth[i] > pKF->mThDepth ||
                            pKF->mvDepth[i] < 0)
                            continue;
                    }

                    nMPs++;
                    if (pMP->Observations() > thObs) {
                        const int& scaleLevel =
                            (pKF->NLeft == -1) ? pKF->mvKeysUn[i].octave
                            : (i < pKF->NLeft) ? pKF->mvKeys[i].octave
                                               : pKF->mvKeysRight[i].octave;
                        const map<KeyFrame*, tuple<int, int>> observations =
                            pMP->GetObservations();
                        int nObs = 0;
                        for (map<KeyFrame*, tuple<int, int>>::const_iterator
                                 mit = observations.begin(),
                                 mend = observations.end();
                             mit != mend; mit++) {
                            KeyFrame* pKFi = mit->first;
                            if (pKFi == pKF) continue;
                            tuple<int, int> indexes = mit->second;
                            int leftIndex = get<0>(indexes),
                                rightIndex = get<1>(indexes);
                            int scaleLeveli = -1;
                            if (pKFi->NLeft == -1)
                                scaleLeveli = pKFi->mvKeysUn[leftIndex].octave;
                            else {
                                if (leftIndex != -1) {
                                    scaleLeveli =
                                        pKFi->mvKeys[leftIndex].octave;
                                }
                                if (rightIndex != -1) {
                                    int rightLevel =
                                        pKFi->mvKeysRight[rightIndex -
                                                          pKFi->NLeft]
                                            .octave;
                                    scaleLeveli = (scaleLeveli == -1 ||
                                                   scaleLeveli > rightLevel)
                                                      ? rightLevel
                                                      : scaleLeveli;
                                }
                            }

                            if (scaleLeveli <= scaleLevel + 1) {
                                nObs++;
                                if (nObs > thObs) break;
                            }
                        }
                        if (nObs > thObs) {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }

        if (nRedundantObservations > redundant_th * nMPs) {
            if (mbInertial) {
                if (mpAtlas->KeyFramesInMap() <= Nd) continue;

                if (pKF->mnId > (mpCurrentKeyFrame->mnId - 2)) continue;

                if (pKF->mPrevKF && pKF->mNextKF) {
                    const float t =
                        pKF->mNextKF->mTimeStamp - pKF->mPrevKF->mTimeStamp;

                    if ((bInitImu && (pKF->mnId < last_ID) && t < 3.) ||
                        (t < 0.5)) {
                        pKF->mNextKF->mpImuPreintegrated->MergePrevious(
                            pKF->mpImuPreintegrated);
                        pKF->mNextKF->mPrevKF = pKF->mPrevKF;
                        pKF->mPrevKF->mNextKF = pKF->mNextKF;
                        pKF->mNextKF = NULL;
                        pKF->mPrevKF = NULL;
                        pKF->SetBadFlag();
                    } else if (!mpCurrentKeyFrame->GetMap()
                                    ->GetIniertialBA2() &&
                               ((pKF->GetImuPosition() -
                                 pKF->mPrevKF->GetImuPosition())
                                    .norm() < 0.02) &&
                               (t < 3)) {
                        pKF->mNextKF->mpImuPreintegrated->MergePrevious(
                            pKF->mpImuPreintegrated);
                        pKF->mNextKF->mPrevKF = pKF->mPrevKF;
                        pKF->mPrevKF->mNextKF = pKF->mNextKF;
                        pKF->mNextKF = NULL;
                        pKF->mPrevKF = NULL;
                        pKF->SetBadFlag();
                    }
                }
            } else {
                pKF->SetBadFlag();
            }
        }
        if ((count > 20 && mbAbortBA) || count > 100) {
            break;
        }
    }
}

}  // namespace ORB_SLAM3
