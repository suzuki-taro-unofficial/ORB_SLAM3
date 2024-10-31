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

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include <Eigen/Core>
#include <fstream>
#include <list>
#include <mutex>

namespace ORB_SLAM3 {

class Atlas;
class KeyFrame;
class LoopClosing;
class Map;
class MapPoint;
class System;
class Tracking;

class LocalMapping {
private:
    /// TODO: Add comment
    void RunOnce();

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LocalMapping(System* pSys, Atlas* pAtlas, const float bMonocular,
                 bool bInertial,
                 const std::string& _strSeqName = std::string());

    void SetLoopCloser(LoopClosing* pLoopCloser);

    void SetTracker(Tracking* pTracker);

    /**
     * 内部でループをしており、以下の動作を行う
     * -
     * 入力キューに入っているキーフレームを前処理しマップに追加(ProcessNewKeyFrame)
     * - 不要なマップポイントを削除(MapPointCulling)
     * - キーフレーム間で3D点の復元(CreateNewMapPoint)
     * - 現在のキーフレーム周辺のキーフレームを用いてBAを行う
     * - IMUの初期化と最適化を行う。
     * - LoopClosingへ現在のキーフレームを渡す。
     */
    void Run();

    /**
     * キーフレームをキューに入れて現在行われているBAを停止させる。
     */
    void InsertKeyFrame(KeyFrame* pKF);

    /**
     * 入力キューが空になるまでProcessNewKeyFrameを行う。
     */
    void EmptyQueue();

    // Thread Synch
    void RequestStop();
    void RequestReset();
    void RequestResetActiveMap(Map* pMap);
    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

    void InterruptBA();

    void RequestFinish();
    bool isFinished();

    int KeyframesInQueue() {
        std::unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

    bool IsInitializing();
    double GetCurrKFTime();
    KeyFrame* GetCurrKF();

    /**
     * System::SaveDebugDataでtxtファイルに出力されるが、
     * 初期化も書き込みもされないため意味はなさそう。
     */
    Eigen::MatrixXd mcovInertial;
    Eigen::Matrix3d mRwg;
    /**
     * current KFから取得したジャイロのバイアスが保持される。
     *
     * LocalMapping::InitializeIMUで書き込まれ、
     * そのままOptimizer::InertialOptimizationに渡される。
     * 書き込みするのに条件がかかっているので、ただのメモではない？
     *
     * あとはSystemによってファイルに保存される
     */
    Eigen::Vector3d mbg;
    /**
     * ほぼ同上、ただし加速度のバイアス
     */
    Eigen::Vector3d mba;
    /**
     * スケール、拡大縮小の情報が入る。
     *
     * 使用する前に必ず1.0で初期化されOptimizerによって最適化された値を使用するため、
     * メンバ変数ではなくローカルな変数に変更可能。SaveDebugDataで読み取られることを除けば
     */
    double mScale;
    /**
     * LocalMapping::InitializeIMUから書き込まれ,SaveDebugDataで読み取られる
     */
    double mInitTime;
    /**
     * 書き込みは行われない。SaveDebugDataで読み取られる
     *
     * 消してよさそう
     */
    double mCostTime;

    unsigned int mInitSect;
    unsigned int mIdxInit;
    unsigned int mnKFs;
    /// Mapで最初のKFの時間
    /// - `InitializeIMU` を実行時に設定され、Systemが読み込む
    /// - Mapの切り替わり時にTracking側からも設定される。
    /// - 両者とも、Mapで最初のKFの時間を設定している。
    /// - Trackingから設定される方が先なはず。
    double mFirstTs;
    int mnMatchesInliers;

    // For debugging (erase in normal mode)
    int mInitFr;
    int mIdxIteration;
    std::string strSequence;

    bool mbNotBA1;
    bool mbNotBA2;
    bool mbBadImu;

    bool mbWriteStats;

    // not consider far points (clouds)
    bool mbFarPoints;
    float mThFarPoints;

protected:
    /**
     * 入力キューにキーフレームがあるならtrueを返す。
     */
    bool CheckNewKeyFrames();

    /**
     * - BoWの計算
     * - MapPointsを新しいキーフレームに関連付け、法線と記述子を更新
     * - Covisibility グラフのリンクを更新する
     * - MapにKeyFrameを挿入
     */
    void ProcessNewKeyFrame();

    void CreateNewMapPoints();

    void MapPointCulling();
    void SearchInNeighbors();
    void KeyFrameCulling();

    System* mpSystem;

    bool mbMonocular;
    bool mbInertial;

    void ResetIfRequested();
    bool mbResetRequested;
    bool mbResetRequestedActiveMap;
    Map* mpMapToReset;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Atlas* mpAtlas;

    LoopClosing* mpLoopCloser;
    Tracking* mpTracker;

    std::list<KeyFrame*> mlNewKeyFrames;

    KeyFrame* mpCurrentKeyFrame;

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    bool mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;

    /**
     * InertialOptimizationとFullInertialBAを行っている
     */
    void InitializeIMU(float priorG = 1e2, float priorA = 1e6,
                       bool bFirst = false);
    void ScaleRefinement();

    bool bInitializing;

    Eigen::MatrixXd infoInertial;
    int mNumLM;
    int mNumKFCulling;

    float mTinit;

    int countRefinement;

    // DEBUG
    std::ofstream f_lm;
};

}  // namespace ORB_SLAM3

#endif  // LOCALMAPPING_H
