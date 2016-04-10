#include "model_server.h"

Eigen::Matrix3d skew(const Eigen::Vector3d &v);

Eigen::Matrix6d getM()
{
    Eigen::Matrix6d M;

    M << 1, 1, 1, 1, 1, 1,
         1, 1, 1, 1, 1, 1,
         1, 1, 1, 1, 1, 1,
         1, 1, 1, 1, 1, 1,
         1, 1, 1, 1, 1, 1,
         1, 1, 1, 1, 1, 1;

    return M;
}

Eigen::Matrix6d getC()
{
    Eigen::Matrix6d C;

    C << 1, 1, 1, 1, 1, 1,
         1, 1, 1, 1, 1, 1,
         1, 1, 1, 1, 1, 1,
         1, 1, 1, 1, 1, 1,
         1, 1, 1, 1, 1, 1,
         1, 1, 1, 1, 1, 1;

    return C;
}

Eigen::Matrix6d getD()
{
    Eigen::Matrix6d D;

    D << 1, 1, 1, 1, 1, 1,
         1, 1, 1, 1, 1, 1,
         1, 1, 1, 1, 1, 1,
         1, 1, 1, 1, 1, 1,
         1, 1, 1, 1, 1, 1,
         1, 1, 1, 1, 1, 1;

    return D;
}

Eigen::Vector6d getG()
{
    Eigen::Vector6d g;

    g << 1,
         1,
         1,
         1,
         1,
         1;

    return g;
}

Eigen::Matrix3d getR(Eigen::Quaterniond q)
{
    return q.toRotationMatrix();
}

Eigen::Matrix<double,4,3> getT(Eigen::Quaterniond q)
{
    Eigen::Matrix<double,4,3> T;
    T << -q.vec().transpose(),
         q.w()*Eigen::MatrixXd::Identity(3,3) + skew(q.vec());

    return T;
}

Eigen::Matrix<double,7,6> getJ(Eigen::Quaterniond q)
{
    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Matrix<double,4,3> T = getT(q);

    Eigen::Matrix<double,7,6> J;
    J << R, Eigen::MatrixXd::Zero(3,3),
         Eigen::MatrixXd::Zero(4,3), T;

    return J;
}

Eigen::Matrix3d skew(const Eigen::Vector3d &v)
{
    Eigen::Matrix3d S;
    S <<  0,    -v(2),  v(1),
          v(2),  0,    -v(0),
         -v(1),  v(0),  0;
    return S;
}
