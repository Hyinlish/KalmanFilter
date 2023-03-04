#ifndef KALMAN_H
#define KALMAN_H

#include <iostream>
#include "eigen/Eigen/Dense"
// #include "matrix.h"

/**
 * 经典卡尔曼滤波——KF
 * 
 * 仅在状态方程和观测方程为线性时适用
*/
class KalmanFilter
{
    public:
        // 状态矩阵
        Eigen::VectorXf X;
        // 观测矩阵
        Eigen::MatrixXf H;
        Eigen::VectorXf Z;
        // 状态变换矩阵
        Eigen::MatrixXf F;
        // 估计误差协方差矩阵
        Eigen::MatrixXf P;
        // 过程噪声协方差矩阵
        Eigen::MatrixXf Q;
        // 测量噪声协方差矩阵
        Eigen::MatrixXf R;
        // 单位矩阵
        Eigen::MatrixXf I;
        // 初始化
        KalmanFilter(int Xsize, int Hsize);
        void init(Eigen::VectorXf X, Eigen::MatrixXf P, Eigen::MatrixXf Q, Eigen::MatrixXf R);
        // 预测
        Eigen::VectorXf predict(Eigen::MatrixXf F);
        // 更新
        // Z为传感器输入值构成的矩阵
        void update(Eigen::MatrixXf H, Eigen::VectorXf Z);
};

#endif