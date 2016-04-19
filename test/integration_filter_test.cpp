#include "ros/ros.h"
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include "maelstrom_msgs/SensorRaw.h"
#include "maelstrom_msgs/State.h"

class IntegrationFilterTest : public ::testing::Test {
public:
    IntegrationFilterTest()
    {
        pub_ = nh_.advertise<maelstrom_msgs::SensorRaw>("sensor", 10);
        sub_ = nh_.subscribe("state_estimate", 10, &IntegrationFilterTest::Callback, this);
        message_received_ = false;
    }

    void SetUp()
    {
        while (!IsNodeReady())
        {
            ros::spinOnce();
        }
    }

    void Publish(Eigen::Vector3d a, Eigen::Vector3d omega)
    {
        maelstrom_msgs::SensorRaw msg;
        tf::vectorEigenToMsg(a, msg.acceleration);
        tf::vectorEigenToMsg(omega, msg.gyro);
        pub_.publish(msg);
    }

    void WaitForMessage()
    {
        while (!message_received_)
        {
            ros::spinOnce();
        }
    }

    Eigen::Vector3d    p_hat_;
    Eigen::Quaterniond q_hat_;
    Eigen::Vector3d    v_hat_;
    Eigen::Vector3d    omega_hat_;
    static const double EPSILON = 1e-6;

 private:
    ros::NodeHandle nh_;
    ros::Publisher  pub_;
    ros::Subscriber sub_;
    bool message_received_;

    void Callback(const maelstrom_msgs::State& msg)
    {
        tf::pointMsgToEigen(msg.pose.position, p_hat_);
        tf::quaternionMsgToEigen(msg.pose.orientation, q_hat_);
        tf::vectorMsgToEigen(msg.twist.linear, v_hat_);
        tf::vectorMsgToEigen(msg.twist.angular, omega_hat_);
        message_received_ = true;
    }

    bool IsNodeReady()
    {
        return (pub_.getNumSubscribers() > 0) && (sub_.getNumPublishers() > 0);
    }
};

TEST_F(IntegrationFilterTest, CheckResponsiveness)
{
    Eigen::Vector3d acceleration;
    Eigen::Vector3d gyro;

    Publish(acceleration, gyro); 
    WaitForMessage();
    EXPECT_TRUE(true);
}

TEST_F(IntegrationFilterTest, ZeroInZeroOut)
{
    Eigen::Vector3d acceleration;
    Eigen::Vector3d gyro;
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "integration_filter_test");

    int ret = RUN_ALL_TESTS();
    ros::shutdown();
    return ret;
}
