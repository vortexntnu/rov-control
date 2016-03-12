#include "ros/ros.h"
#include "geometry_msgs/Twist.h" // Not sure if necessary
#include "geometry_msgs/Pose.h"  // Ditto
#include "uranus_dp/GetM.h"
#include "uranus_dp/GetC.h"
#include "uranus_dp/GetD.h"
#include "uranus_dp/GetG.h"
#include "uranus_dp/GetR.h"
#include "uranus_dp/GetT.h"
#include <eigen_conversions/eigen_msg.h>

Eigen::Matrix3d skew(const Eigen::Vector3d &v);

bool getM(uranus_dp::GetM::Request  &req,
          uranus_dp::GetM::Response &resp)
{
    // 1st row
    resp.M[0] = 1;
    resp.M[1] = 1;
    resp.M[2] = 1;
    resp.M[3] = 1;
    resp.M[4] = 1;
    resp.M[5] = 1;

    // 2nd row
    resp.M[6]  = 1;
    resp.M[7]  = 1;
    resp.M[8]  = 1;
    resp.M[9]  = 1;
    resp.M[10] = 1;
    resp.M[11] = 1;

    // 3rd row
    resp.M[12] = 1;
    resp.M[13] = 1;
    resp.M[14] = 1;
    resp.M[15] = 1;
    resp.M[16] = 1;
    resp.M[17] = 1;

    // 4th row
    resp.M[18] = 1;
    resp.M[19] = 1;
    resp.M[20] = 1;
    resp.M[21] = 1;
    resp.M[22] = 1;
    resp.M[23] = 1;

    // 5th row
    resp.M[24] = 1;
    resp.M[25] = 1;
    resp.M[26] = 1;
    resp.M[27] = 1;
    resp.M[28] = 1;
    resp.M[29] = 1;

    // 6th row
    resp.M[30] = 1;
    resp.M[31] = 1;
    resp.M[32] = 1;
    resp.M[33] = 1;
    resp.M[34] = 1;
    resp.M[35] = 1;

    return true;
}

bool getC(uranus_dp::GetC::Request  &req,
          uranus_dp::GetC::Response &resp)
{
    // 1st row
    resp.C[0] = 1;
    resp.C[1] = 1;
    resp.C[2] = 1;
    resp.C[3] = 1;
    resp.C[4] = 1;
    resp.C[5] = 1;

    // 2nd row
    resp.C[6]  = 1;
    resp.C[7]  = 1;
    resp.C[8]  = 1;
    resp.C[9]  = 1;
    resp.C[10] = 1;
    resp.C[11] = 1;

    // 3rd row
    resp.C[12] = 1;
    resp.C[13] = 1;
    resp.C[14] = 1;
    resp.C[15] = 1;
    resp.C[16] = 1;
    resp.C[17] = 1;

    // 4th row
    resp.C[18] = 1;
    resp.C[19] = 1;
    resp.C[20] = 1;
    resp.C[21] = 1;
    resp.C[22] = 1;
    resp.C[23] = 1;

    // 5th row
    resp.C[24] = 1;
    resp.C[25] = 1;
    resp.C[26] = 1;
    resp.C[27] = 1;
    resp.C[28] = 1;
    resp.C[29] = 1;

    // 6th row
    resp.C[30] = 1;
    resp.C[31] = 1;
    resp.C[32] = 1;
    resp.C[33] = 1;
    resp.C[34] = 1;
    resp.C[35] = 1;

    return true;
}

bool getD(uranus_dp::GetD::Request  &req,
          uranus_dp::GetD::Response &resp)
{
    // 1st row
    resp.D[0] = 1;
    resp.D[1] = 1;
    resp.D[2] = 1;
    resp.D[3] = 1;
    resp.D[4] = 1;
    resp.D[5] = 1;

    // 2nd row
    resp.D[6]  = 1;
    resp.D[7]  = 1;
    resp.D[8]  = 1;
    resp.D[9]  = 1;
    resp.D[10] = 1;
    resp.D[11] = 1;

    // 3rd row
    resp.D[12] = 1;
    resp.D[13] = 1;
    resp.D[14] = 1;
    resp.D[15] = 1;
    resp.D[16] = 1;
    resp.D[17] = 1;

    // 4th row
    resp.D[18] = 1;
    resp.D[19] = 1;
    resp.D[20] = 1;
    resp.D[21] = 1;
    resp.D[22] = 1;
    resp.D[23] = 1;

    // 5th row
    resp.D[24] = 1;
    resp.D[25] = 1;
    resp.D[26] = 1;
    resp.D[27] = 1;
    resp.D[28] = 1;
    resp.D[29] = 1;

    // 6th row
    resp.D[30] = 1;
    resp.D[31] = 1;
    resp.D[32] = 1;
    resp.D[33] = 1;
    resp.D[34] = 1;
    resp.D[35] = 1;

    return true;
}

bool getG(uranus_dp::GetG::Request  &req,
          uranus_dp::GetG::Response &resp)
{
    resp.g[0] = 1;
    resp.g[1] = 1;
    resp.g[2] = 1;
    resp.g[3] = 1;
    resp.g[4] = 1;
    resp.g[5] = 1;

    return true;
}

bool getR(uranus_dp::GetR::Request  &req,
          uranus_dp::GetR::Response &resp)
{
    Eigen::Quaterniond q;
    tf::quaternionMsgToEigen(req.q, q);

    Eigen::Matrix3d R = q.matrix();

    // 1st row
    resp.R[0] = R(0);
    resp.R[1] = R(1);
    resp.R[2] = R(2);

    // 2nd row
    resp.R[3] = R(3);
    resp.R[4] = R(4);
    resp.R[5] = R(5);

    // 3rd row
    resp.R[6] = R(6);
    resp.R[7] = R(7);
    resp.R[8] = R(8);

    return true;
}

bool getT(uranus_dp::GetT::Request  &req,
          uranus_dp::GetT::Response &resp)
{
    Eigen::Quaterniond q;
    tf::quaternionMsgToEigen(req.q, q);

    Eigen::Matrix<double,4,3> T;

    T << -q.vec().transpose(),
         q.w()*Eigen::MatrixXd::Identity(3,3) + skew(q.vec());

    // 1st row
    resp.T[0] = T(0);
    resp.T[1] = T(1);
    resp.T[2] = T(2);

    // 2nd row
    resp.T[3] = T(3);
    resp.T[4] = T(4);
    resp.T[5] = T(5);

    // 3rd row
    resp.T[6] = T(6);
    resp.T[7] = T(7);
    resp.T[8] = T(8);

    // 4th row
    resp.T[9]  = T(9);
    resp.T[10] = T(10);
    resp.T[11] = T(11);

    return true;
}

Eigen::Matrix3d skew(const Eigen::Vector3d &v)
{
    Eigen::Matrix3d S;
    S << 0, -v(2), v(1),
         v(2), 0, -v(0),
         -v(1), v(0), 0;
    return S;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "model_server");
    ros::NodeHandle nh;

    ros::ServiceServer serviceM = nh.advertiseService("get_m", getM);
    ros::ServiceServer serviceC = nh.advertiseService("get_c", getC);
    ros::ServiceServer serviceD = nh.advertiseService("get_d", getD);
    ros::ServiceServer serviceG = nh.advertiseService("get_g", getG);
    ros::ServiceServer serviceR = nh.advertiseService("get_r", getR);
    ros::spin();

    return 0;
}
