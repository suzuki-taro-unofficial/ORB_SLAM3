#include "Atlas.h"
#include "GeometricCamera.h"
#include "GeometricTools.h"
#include "LocalMapping.h"
#include "Map.h"
#include "ORBmatcher.h"
#include "Tracking.h"

namespace ORB_SLAM3 {

void LocalMapping::CreateNewMapPoints() {
    // Retrieve neighbor keyframes in covisibility graph
    // retrieve 取り出す
    int nn = 10;
    // For stereo inertial case
    if (mbMonocular) nn = 30;

    vector<KeyFrame*> vpNeighKFs =
        mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
    // prevKFを探索していってvpNeighKFsに含まれていたらvpNeighKFsに追加する。
    // つまりダブりを生んでる。意図は良くわからない
    if (mbInertial) {
        KeyFrame* pKF = mpCurrentKeyFrame;
        int count = 0;
        while ((vpNeighKFs.size() <= nn) && (pKF->mPrevKF) && (count++ < nn)) {
            vector<KeyFrame*>::iterator it =
                std::find(vpNeighKFs.begin(), vpNeighKFs.end(), pKF->mPrevKF);
            if (it == vpNeighKFs.end()) vpNeighKFs.push_back(pKF->mPrevKF);
            pKF = pKF->mPrevKF;
        }
    }

    float th = 0.6f;

    ORBmatcher matcher(th, false);

    Sophus::SE3<float> sophTcw1 = mpCurrentKeyFrame->GetPose();
    Eigen::Matrix<float, 3, 4> eigTcw1 = sophTcw1.matrix3x4();
    Eigen::Matrix<float, 3, 3> Rcw1 = eigTcw1.block<3, 3>(0, 0);
    Eigen::Matrix<float, 3, 3> Rwc1 = Rcw1.transpose();
    Eigen::Vector3f tcw1 = sophTcw1.translation();
    Eigen::Vector3f Ow1 = mpCurrentKeyFrame->GetCameraCenter();

    const float& fx1 = mpCurrentKeyFrame->fx;
    const float& fy1 = mpCurrentKeyFrame->fy;
    const float& cx1 = mpCurrentKeyFrame->cx;
    const float& cy1 = mpCurrentKeyFrame->cy;

    const float ratioFactor = 1.5f * mpCurrentKeyFrame->mfScaleFactor;
    // Search matches with epipolar restriction and triangulate
    for (size_t i = 0; i < vpNeighKFs.size(); i++) {
        if (i > 0 && CheckNewKeyFrames()) return;

        KeyFrame* pKF2 = vpNeighKFs[i];

        GeometricCamera *pCamera1 = mpCurrentKeyFrame->mpCamera,
                        *pCamera2 = pKF2->mpCamera;

        // Check first that baseline is not too short
        Eigen::Vector3f Ow2 = pKF2->GetCameraCenter();
        Eigen::Vector3f vBaseline = Ow2 - Ow1;
        const float baseline = vBaseline.norm();

        if (!mbMonocular) {
            if (baseline < pKF2->mb) continue;
        } else {
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
            const float ratioBaselineDepth = baseline / medianDepthKF2;

            if (ratioBaselineDepth < 0.01) continue;
        }

        // Search matches that fullfil epipolar constraint
        // epipolar constraint エピポーラ制約
        vector<pair<size_t, size_t>> vMatchedIndices;
        bool bCoarse = mbInertial &&
                       mpTracker->mState == Tracking::RECENTLY_LOST &&
                       mpCurrentKeyFrame->GetMap()->GetIniertialBA2();

        matcher.SearchForTriangulation(mpCurrentKeyFrame, pKF2, vMatchedIndices,
                                       false, bCoarse);

        Sophus::SE3<float> sophTcw2 = pKF2->GetPose();
        Eigen::Matrix<float, 3, 4> eigTcw2 = sophTcw2.matrix3x4();
        Eigen::Matrix<float, 3, 3> Rcw2 = eigTcw2.block<3, 3>(0, 0);
        Eigen::Matrix<float, 3, 3> Rwc2 = Rcw2.transpose();
        Eigen::Vector3f tcw2 = sophTcw2.translation();

        const float& fx2 = pKF2->fx;
        const float& fy2 = pKF2->fy;
        const float& cx2 = pKF2->cx;
        const float& cy2 = pKF2->cy;

        // Triangulate each match
        const int nmatches = vMatchedIndices.size();
        for (int ikp = 0; ikp < nmatches; ikp++) {
            const int& idx1 = vMatchedIndices[ikp].first;
            const int& idx2 = vMatchedIndices[ikp].second;

            const cv::KeyPoint& kp1 =
                (mpCurrentKeyFrame->NLeft == -1)
                    ? mpCurrentKeyFrame->mvKeysUn[idx1]
                : (idx1 < mpCurrentKeyFrame->NLeft)
                    ? mpCurrentKeyFrame->mvKeys[idx1]
                    : mpCurrentKeyFrame
                          ->mvKeysRight[idx1 - mpCurrentKeyFrame->NLeft];
            const float kp1_ur = mpCurrentKeyFrame->mvuRight[idx1];
            bool bStereo1 = (!mpCurrentKeyFrame->mpCamera2 && kp1_ur >= 0);
            const bool bRight1 = (mpCurrentKeyFrame->NLeft == -1 ||
                                  idx1 < mpCurrentKeyFrame->NLeft)
                                     ? false
                                     : true;

            const cv::KeyPoint& kp2 =
                (pKF2->NLeft == -1)    ? pKF2->mvKeysUn[idx2]
                : (idx2 < pKF2->NLeft) ? pKF2->mvKeys[idx2]
                                       : pKF2->mvKeysRight[idx2 - pKF2->NLeft];

            const float kp2_ur = pKF2->mvuRight[idx2];
            bool bStereo2 = (!pKF2->mpCamera2 && kp2_ur >= 0);
            const bool bRight2 =
                (pKF2->NLeft == -1 || idx2 < pKF2->NLeft) ? false : true;

            if (mpCurrentKeyFrame->mpCamera2 && pKF2->mpCamera2) {
                if (bRight1 && bRight2) {
                    sophTcw1 = mpCurrentKeyFrame->GetRightPose();
                    Ow1 = mpCurrentKeyFrame->GetRightCameraCenter();

                    sophTcw2 = pKF2->GetRightPose();
                    Ow2 = pKF2->GetRightCameraCenter();

                    pCamera1 = mpCurrentKeyFrame->mpCamera2;
                    pCamera2 = pKF2->mpCamera2;
                } else if (bRight1 && !bRight2) {
                    sophTcw1 = mpCurrentKeyFrame->GetRightPose();
                    Ow1 = mpCurrentKeyFrame->GetRightCameraCenter();

                    sophTcw2 = pKF2->GetPose();
                    Ow2 = pKF2->GetCameraCenter();

                    pCamera1 = mpCurrentKeyFrame->mpCamera2;
                    pCamera2 = pKF2->mpCamera;
                } else if (!bRight1 && bRight2) {
                    sophTcw1 = mpCurrentKeyFrame->GetPose();
                    Ow1 = mpCurrentKeyFrame->GetCameraCenter();

                    sophTcw2 = pKF2->GetRightPose();
                    Ow2 = pKF2->GetRightCameraCenter();

                    pCamera1 = mpCurrentKeyFrame->mpCamera;
                    pCamera2 = pKF2->mpCamera2;
                } else {
                    sophTcw1 = mpCurrentKeyFrame->GetPose();
                    Ow1 = mpCurrentKeyFrame->GetCameraCenter();

                    sophTcw2 = pKF2->GetPose();
                    Ow2 = pKF2->GetCameraCenter();

                    pCamera1 = mpCurrentKeyFrame->mpCamera;
                    pCamera2 = pKF2->mpCamera;
                }
                eigTcw1 = sophTcw1.matrix3x4();
                Rcw1 = eigTcw1.block<3, 3>(0, 0);
                Rwc1 = Rcw1.transpose();
                tcw1 = sophTcw1.translation();

                eigTcw2 = sophTcw2.matrix3x4();
                Rcw2 = eigTcw2.block<3, 3>(0, 0);
                Rwc2 = Rcw2.transpose();
                tcw2 = sophTcw2.translation();
            }

            // Check parallax between rays
            Eigen::Vector3f xn1 = pCamera1->unprojectEig(kp1.pt);
            Eigen::Vector3f xn2 = pCamera2->unprojectEig(kp2.pt);

            Eigen::Vector3f ray1 = Rwc1 * xn1;
            Eigen::Vector3f ray2 = Rwc2 * xn2;
            const float cosParallaxRays =
                ray1.dot(ray2) / (ray1.norm() * ray2.norm());

            float cosParallaxStereo = cosParallaxRays + 1;
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;

            if (bStereo1)
                cosParallaxStereo1 =
                    cos(2 * atan2(mpCurrentKeyFrame->mb / 2,
                                  mpCurrentKeyFrame->mvDepth[idx1]));
            else if (bStereo2)
                cosParallaxStereo2 =
                    cos(2 * atan2(pKF2->mb / 2, pKF2->mvDepth[idx2]));

            cosParallaxStereo = min(cosParallaxStereo1, cosParallaxStereo2);

            Eigen::Vector3f x3D;

            bool goodProj = false;
            if (cosParallaxRays < cosParallaxStereo && cosParallaxRays > 0 &&
                (bStereo1 || bStereo2 ||
                 (cosParallaxRays < 0.9996 && mbInertial) ||
                 (cosParallaxRays < 0.9998 && !mbInertial))) {
                goodProj = GeometricTools::Triangulate(xn1, xn2, eigTcw1,
                                                       eigTcw2, x3D);
                if (!goodProj) continue;
            } else if (bStereo1 && cosParallaxStereo1 < cosParallaxStereo2) {
                goodProj = mpCurrentKeyFrame->UnprojectStereo(idx1, x3D);
            } else if (bStereo2 && cosParallaxStereo2 < cosParallaxStereo1) {
                goodProj = pKF2->UnprojectStereo(idx2, x3D);
            } else {
                continue;  // No stereo and very low parallax
            }

            if (!goodProj) continue;

            // Check triangulation in front of cameras
            float z1 = Rcw1.row(2).dot(x3D) + tcw1(2);
            if (z1 <= 0) continue;

            float z2 = Rcw2.row(2).dot(x3D) + tcw2(2);
            if (z2 <= 0) continue;

            // Check reprojection error in first keyframe
            const float& sigmaSquare1 =
                mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
            const float x1 = Rcw1.row(0).dot(x3D) + tcw1(0);
            const float y1 = Rcw1.row(1).dot(x3D) + tcw1(1);
            const float invz1 = 1.0 / z1;

            if (!bStereo1) {
                cv::Point2f uv1 = pCamera1->project(cv::Point3f(x1, y1, z1));
                float errX1 = uv1.x - kp1.pt.x;
                float errY1 = uv1.y - kp1.pt.y;

                if ((errX1 * errX1 + errY1 * errY1) > 5.991 * sigmaSquare1)
                    continue;

            } else {
                float u1 = fx1 * x1 * invz1 + cx1;
                float u1_r = u1 - mpCurrentKeyFrame->mbf * invz1;
                float v1 = fy1 * y1 * invz1 + cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                float errX1_r = u1_r - kp1_ur;
                if ((errX1 * errX1 + errY1 * errY1 + errX1_r * errX1_r) >
                    7.8 * sigmaSquare1)
                    continue;
            }

            // Check reprojection error in second keyframe
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
            const float x2 = Rcw2.row(0).dot(x3D) + tcw2(0);
            const float y2 = Rcw2.row(1).dot(x3D) + tcw2(1);
            const float invz2 = 1.0 / z2;
            if (!bStereo2) {
                cv::Point2f uv2 = pCamera2->project(cv::Point3f(x2, y2, z2));
                float errX2 = uv2.x - kp2.pt.x;
                float errY2 = uv2.y - kp2.pt.y;
                if ((errX2 * errX2 + errY2 * errY2) > 5.991 * sigmaSquare2)
                    continue;
            } else {
                float u2 = fx2 * x2 * invz2 + cx2;
                float u2_r = u2 - mpCurrentKeyFrame->mbf * invz2;
                float v2 = fy2 * y2 * invz2 + cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                float errX2_r = u2_r - kp2_ur;
                if ((errX2 * errX2 + errY2 * errY2 + errX2_r * errX2_r) >
                    7.8 * sigmaSquare2)
                    continue;
            }

            // Check scale consistency
            Eigen::Vector3f normal1 = x3D - Ow1;
            float dist1 = normal1.norm();

            Eigen::Vector3f normal2 = x3D - Ow2;
            float dist2 = normal2.norm();

            if (dist1 == 0 || dist2 == 0) continue;

            if (mbFarPoints && (dist1 >= mThFarPoints ||
                                dist2 >= mThFarPoints))  // MODIFICATION
                continue;

            const float ratioDist = dist2 / dist1;
            const float ratioOctave =
                mpCurrentKeyFrame->mvScaleFactors[kp1.octave] /
                pKF2->mvScaleFactors[kp2.octave];

            if (ratioDist * ratioFactor < ratioOctave ||
                ratioDist > ratioOctave * ratioFactor)
                continue;

            // Triangulation is succesfull
            MapPoint* pMP =
                new MapPoint(x3D, mpCurrentKeyFrame, mpAtlas->GetCurrentMap());

            pMP->AddObservation(mpCurrentKeyFrame, idx1);
            pMP->AddObservation(pKF2, idx2);

            mpCurrentKeyFrame->AddMapPoint(pMP, idx1);
            pKF2->AddMapPoint(pMP, idx2);

            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();

            mpAtlas->AddMapPoint(pMP);
            mlpRecentAddedMapPoints.push_back(pMP);
        }
    }
}
}  // namespace ORB_SLAM3
