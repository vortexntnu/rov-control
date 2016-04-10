#include "ros/ros.h"
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include "uranus_dp/State.h"
#include "uranus_dp/ToggleControlMode.h"

class OpenLoopControllerTest : public ::testing::Test {
public:
    OpenLoopControllerTest()
    {
        pub      = nh.advertise<geometry_msgs::Wrench>("open_loop_setpoint", 1);
        sub        = nh.subscribe("control_input", 5, &OpenLoopControllerTest::Callback, this);
        // modeClient         = nh.serviceClient<uranus_dp::ToggleControlMode>("toggle_control_mode");

        message_received = false;
    }

    void SetUp()
    {
        while (!IsNodeReady())
        {
            ros::spinOnce();
        }
    }

    void PublishSetpoint(double X, double Y, double Z, double K, double M, double N)
    {
        geometry_msgs::Wrench msg;
        msg.force.x = X;
        msg.force.y = Y;
        msg.force.z = Z;
        msg.torque.x = K;
        msg.torque.y = M;
        msg.torque.z = N;
        pub.publish(msg);
    }

    void WaitForMessage()
    {
        while (!message_received)
        {
            ros::spinOnce();
        }
    }

    // ros::ServiceClient modeClient;
    Eigen::Matrix<double,6,1> tau;
    static const double EPSILON = 1e-6; // Max absolute error

 private:
    ros::NodeHandle nh;
    ros::Publisher  pub;
    ros::Subscriber sub;

    bool message_received;

    void Callback(const geometry_msgs::Wrench& msg)
    {
        tf::wrenchMsgToEigen(msg, tau);
        message_received = true;
    }

    bool IsNodeReady()
    {
        return (pub.getNumSubscribers() > 0) && (sub.getNumPublishers() > 0);
    }
};

 /*
  * Make sure we receive any messages at all.
  */
TEST_F(OpenLoopControllerTest, CheckResponsiveness)
{
    PublishSetpoint(1,2,3,4,5,6);
    WaitForMessage();
    EXPECT_TRUE(true);
}

 /*
  * Make sure open loop outputs the same as the input.
  */
TEST_F(OpenLoopControllerTest, DirectFeedthrough)
{
    PublishSetpoint(-9.966, -7.907, 7.626, -6.023, 1.506, 2.805);
    WaitForMessage();
    EXPECT_NEAR(tau(0), -9.966, EPSILON);
    EXPECT_NEAR(tau(1), -7.907, EPSILON);
    EXPECT_NEAR(tau(2),  7.626, EPSILON);
    EXPECT_NEAR(tau(3), -6.023, EPSILON);
    EXPECT_NEAR(tau(4),  1.506, EPSILON);
    EXPECT_NEAR(tau(5),  2.805, EPSILON);
}

 /*
  * Input zero speed and nonzero pose. Input same pose state and pose setpoint. Expect zero output.
  */
// TEST_F(OpenLoopControllerTest, ZeroErrorZeroOutput)
// {
//     uranus_dp::ToggleControlMode srv;
//     if (!modeClient.call(srv))
//     {
//         ROS_ERROR_STREAM("Failed to call service toggle_control_mode. New mode not activated.");
//     }

//     Eigen::Vector3d    p(1,2,3);
//     Eigen::Quaterniond q(4,5,6,7);
//     q.normalize();

//     Eigen::Vector3d v(0,0,0);
//     Eigen::Vector3d omega(0,0,0);

//     PublishState(p, q, v, omega);
//     PublishStationkeeperSetpoint(p, q);

//     WaitForMessage();

//     EXPECT_NEAR(tau(0), 0, EPSILON);
//     EXPECT_NEAR(tau(1), 0, EPSILON);
//     EXPECT_NEAR(tau(2), 0, EPSILON);
//     EXPECT_NEAR(tau(3), 0, EPSILON);
//     EXPECT_NEAR(tau(4), 0, EPSILON);
//     EXPECT_NEAR(tau(5), 0, EPSILON);
// }

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "controller_test");
    ROS_INFO("Launching node controller_test.");

    int ret = RUN_ALL_TESTS();
    ros::shutdown();
    return ret;
}
