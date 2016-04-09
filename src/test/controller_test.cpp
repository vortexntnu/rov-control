#include "ros/ros.h"
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include "geometry_msgs/Wrench.h"
#include "uranus_dp/State.h"

class ControllerTest : public ::testing::Test {
public:
    ControllerTest()
    {
        state_pub_     = node_handle_.advertise<uranus_dp::State>("state", 1);
        setpoint_pub_  = node_handle_.advertise<uranus_dp::State>("setpoint", 1);
        subscriber_    = node_handle_.subscribe("control_input", 5, &ControllerTest::Callback, this);
        message_received_ = false;
    }

    void SetUp()
    {
        while (!IsNodeReady())
        {
            ros::spinOnce();
        }
    }

    void PublishState(Eigen::Vector3d p, Eigen::Quaterniond q, Eigen::Vector3d v, Eigen::Vector3d omega)
    {
        uranus_dp::State msg;
        msg.pose.position.x    = p(0);
        msg.pose.position.y    = p(1);
        msg.pose.position.z    = p(2);
        msg.pose.orientation.x = q.w();
        msg.pose.orientation.y = q.x();
        msg.pose.orientation.z = q.y();
        msg.pose.orientation.w = q.z();
        msg.twist.linear.x     = v(0);
        msg.twist.linear.y     = v(1);
        msg.twist.linear.z     = v(2);
        msg.twist.angular.x    = omega(0);
        msg.twist.angular.y    = omega(1);
        msg.twist.angular.z    = omega(2);
        state_pub_.publish(msg);
    }

    void PublishSetpoint(Eigen::Vector3d p, Eigen::Quaterniond q, Eigen::Vector3d v, Eigen::Vector3d omega)
    {
        uranus_dp::State msg;
        msg.pose.position.x    = p(0);
        msg.pose.position.y    = p(1);
        msg.pose.position.z    = p(2);
        msg.pose.orientation.x = q.w();
        msg.pose.orientation.y = q.x();
        msg.pose.orientation.z = q.y();
        msg.pose.orientation.w = q.z();
        msg.twist.linear.x     = v(0);
        msg.twist.linear.y     = v(1);
        msg.twist.linear.z     = v(2);
        msg.twist.angular.x    = omega(0);
        msg.twist.angular.y    = omega(1);
        msg.twist.angular.z    = omega(2);
        setpoint_pub_.publish(msg);
    }

    void WaitForMessage()
    {
        message_received_ = false;
        while (!message_received_)
        {
            ros::spinOnce();
        }
    }

    Eigen::Matrix<double,6,1> tau_;
    static const double EPSILON = 1e-6; // Max absolute error (Newton or Newton meters)

 private:
    ros::NodeHandle node_handle_;
    ros::Publisher  state_pub_;
    ros::Publisher  setpoint_pub_;
    ros::Subscriber subscriber_;
    bool message_received_;

    void Callback(const geometry_msgs::Wrench& msg)
    {
        tf::wrenchMsgToEigen(msg, tau_);
        message_received_ = true;
    }

    bool IsNodeReady()
    {
        return (state_pub_.getNumSubscribers() > 0) && (setpoint_pub_.getNumSubscribers() > 0) && (subscriber_.getNumPublishers() > 0);
    }
};

 /*
  * Make sure we receive messages at all.
  */
TEST_F(ControllerTest, CheckResponsiveness)
{
    WaitForMessage();
    ASSERT_TRUE(true);
}

 /*
  * Input zero speed and nonzero pose. Input same pose state and pose setpoint. Expect zero output.
  */
TEST_F(ControllerTest, ZeroErrorZeroOutput)
{
    Eigen::Vector3d    p(1,2,3);
    Eigen::Quaterniond q(4,5,6,7);
    q.normalize();

    Eigen::Vector3d v(0,0,0);
    Eigen::Vector3d omega(0,0,0);

    PublishState(p, q, v, omega);
    PublishSetpoint(p, q, v, omega);

    WaitForMessage();

    EXPECT_NEAR(tau_(0), 0, EPSILON);
    EXPECT_NEAR(tau_(1), 0, EPSILON);
    EXPECT_NEAR(tau_(2), 0, EPSILON);
    EXPECT_NEAR(tau_(3), 0, EPSILON);
    EXPECT_NEAR(tau_(4), 0, EPSILON);
    EXPECT_NEAR(tau_(5), 0, EPSILON);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "controller_test");
    ROS_INFO("Launching node controller_test.");

    int ret = RUN_ALL_TESTS();
    ros::shutdown();
    return ret;
}
