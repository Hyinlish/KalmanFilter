#include "kalman.h"

int main()
{
    Eigen::VectorXf X(2);
    Eigen::MatrixXf F(2, 2);
    Eigen::MatrixXf mP(2, 2);
    Eigen::MatrixXf mQ(2, 2);
    Eigen::MatrixXf mR(2, 2);
    Eigen::MatrixXf H(2, 2);
    float dt = 0.001;

    F <<
    1,dt,
    0,0;

    mP << 
    0.5,0,
    0,0.5;

    mQ <<
    0.005,0,
    0,0.005;

    mR <<
    0.3,0,
    0,0.3;

    H <<
    1,0,
    0,0;

    // 定义一个测试数据集
    float testdata[16] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
    X << testdata[0], 0;

    KalmanFilter MoveKal(2, 1);
    MoveKal.init(X, mP, mQ, mR);

    for(int i=1; i<16; i++)
    {
        Eigen::VectorXf Z(2);
        Z << testdata[i], 0;

        std::cout << "第" << i << "次迭代:" << MoveKal.predict(F) << std::endl;

        MoveKal.update(H, Z);
    }

    return 0;
}