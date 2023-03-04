#include "kalman.h"

KalmanFilter::KalmanFilter(int Xsize, int Hsize)
{
    if(Xsize <= 0 || Hsize <= 0)
    {
        std::cout << "Bad Matrix Size!" << std::endl;
    }
    X.resize(Xsize);
    X.setZero();

    H.resize(Hsize, Xsize);
    H.setIdentity();

    Z.resize(Hsize);
    Z.setZero();

    F.resize(Xsize, Xsize);
    F.setIdentity();

    P.resize(Xsize, Xsize);
    P.setIdentity();

    Q.resize(Xsize, Xsize);
    Q.setIdentity();

    R.resize(Xsize, Xsize);
    R.setIdentity();

    I = Eigen::MatrixXf::Identity(Xsize, Xsize);

    std::cout << "KalmanFilter Created Successfully!" << std::endl;
}

void KalmanFilter::init(Eigen::VectorXf X, Eigen::MatrixXf P, Eigen::MatrixXf Q, Eigen::MatrixXf R)
{
    this->X = X;
    this->P = P;
    this->Q = Q;
    this->R = R;
}

Eigen::VectorXf KalmanFilter::predict(Eigen::MatrixXf F)
{
    X = F * X;
    // 取F的转置
    Eigen::MatrixXf F_T = F.transpose();
    P = F * P * F_T + Q;
    return X;
}

void KalmanFilter::update(Eigen::MatrixXf _H, Eigen::VectorXf _Z)
{
    H = _H;
    Z = _Z;
    Eigen::MatrixXf H_T = H.transpose();
    Eigen::MatrixXf temp1 = H * P * H_T + R; 
    // 更新卡尔曼因数——Kalman Gain
    Eigen::MatrixXf K = P * H_T * temp1.inverse();  // 求矩阵的逆
    // 最终估计结果
    X = X + K * (H * X - Z);
    // 更新估计噪声协方差矩阵
    P = (I - K * H) * P;
}