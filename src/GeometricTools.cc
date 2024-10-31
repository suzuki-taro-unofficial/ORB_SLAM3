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

#include "GeometricTools.h"

#include "GeometricCamera.h"
#include "KeyFrame.h"

namespace ORB_SLAM3 {

/**
 * Chat GPTからの出力
 * 2つのキーフレーム (`KeyFrame`) 間の **基本行列** （Fundamental
 * Matrix）を計算します。
 * 基本行列は2つの異なるカメラ位置からの対応点の関係を表す重要な幾何学的な行列です。
 */
Eigen::Matrix3f GeometricTools::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2) {
    Sophus::SE3<float> Tc1w = pKF1->GetPose();
    Sophus::Matrix3<float> Rc1w = Tc1w.rotationMatrix();
    Sophus::SE3<float>::TranslationMember tc1w = Tc1w.translation();

    Sophus::SE3<float> Tc2w = pKF2->GetPose();
    Sophus::Matrix3<float> Rc2w = Tc2w.rotationMatrix();
    Sophus::SE3<float>::TranslationMember tc2w = Tc2w.translation();

    Sophus::Matrix3<float> Rc1c2 = Rc1w * Rc2w.transpose();
    Eigen::Vector3f tc1c2 = -Rc1c2 * tc2w + tc1w;

    Eigen::Matrix3f tc1c2x = Sophus::SO3f::hat(tc1c2);

    const Eigen::Matrix3f K1 = pKF1->mpCamera->toK_();
    const Eigen::Matrix3f K2 = pKF2->mpCamera->toK_();

    return K1.transpose().inverse() * tc1c2x * Rc1c2 * K2.inverse();
}

/**
 * Chat GPT からの出力
 * 2つのカメラ座標系 (`x_c1`, `x_c2`) のポイントを使って、3次元空間上の点
 * (`x3D`) を **三角測量** します。
 * この手法は、2つの異なる視点からの2次元の画像データを元にして3次元の位置を計算するために使用されます。
 */
bool GeometricTools::Triangulate(Eigen::Vector3f &x_c1, Eigen::Vector3f &x_c2,
                                 Eigen::Matrix<float, 3, 4> &Tc1w,
                                 Eigen::Matrix<float, 3, 4> &Tc2w,
                                 Eigen::Vector3f &x3D) {
    Eigen::Matrix4f A;
    A.block<1, 4>(0, 0) =
        x_c1(0) * Tc1w.block<1, 4>(2, 0) - Tc1w.block<1, 4>(0, 0);
    A.block<1, 4>(1, 0) =
        x_c1(1) * Tc1w.block<1, 4>(2, 0) - Tc1w.block<1, 4>(1, 0);
    A.block<1, 4>(2, 0) =
        x_c2(0) * Tc2w.block<1, 4>(2, 0) - Tc2w.block<1, 4>(0, 0);
    A.block<1, 4>(3, 0) =
        x_c2(1) * Tc2w.block<1, 4>(2, 0) - Tc2w.block<1, 4>(1, 0);

    Eigen::JacobiSVD<Eigen::Matrix4f> svd(A, Eigen::ComputeFullV);

    Eigen::Vector4f x3Dh = svd.matrixV().col(3);

    if (x3Dh(3) == 0) return false;

    // Euclidean coordinates
    x3D = x3Dh.head(3) / x3Dh(3);

    return true;
}

}  // namespace ORB_SLAM3
