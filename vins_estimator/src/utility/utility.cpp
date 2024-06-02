/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-06-02 11:09:58
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-06-02 11:09:59
 * @FilePath: /catkin_ws_vins_fusion/media/psf/develop/code/autopilot/slam/VINS-Fusion/vins_estimator/src/utility/utility.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "utility.h"


/**
 * @brief g2R：gravity To world 将IMU的Z轴与重力方向对齐，作为初始位姿R0
 *        此时R0作为初始位姿或许不准确，但能提供初值即可
 * @param[in] g 平均得到的重力值
 * @param[out] R0 
*/
Eigen::Matrix3d Utility::g2R(const Eigen::Vector3d &g)
{
    Eigen::Matrix3d R0;
    Eigen::Vector3d ng1 = g.normalized();                                                                                          // g归一化
    Eigen::Vector3d ng2{0, 0, 1.0};
    // 求解从：g 变换到 {0, 0, 1.0} 的旋转                                                                                                           // 单位方向
    R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
    double yaw = Utility::R2ypr(R0).x();
    // 取了yaw的负值，表示从 {0, 0, 1.0} 到 g 绕Z轴的旋转角度
    // 这里转回去，表示，不希望发生有围绕Z轴的旋转
    // 第一帧yaw角不可观，置零即可
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;                                                                // 将yaw取出来，再进行二次对齐
    // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
    return R0;
}
