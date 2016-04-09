#include "ros/ros.h"
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "geometry_msgs/Wrench.h"
#include "uranus_dp/State.h"

class ControllerTest : public ::testing::Test {
public:
    ControllerTest()
    {
        state_pub_     = node_handle_.advertise<uranus_dp::State>("state", 5);
        setpoint_pub_  = node_handle_.advertise<uranus_dp::State>("setpoint", 5);
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

    // void PublishState(Eigen::Vector3 p, Eigen::Quaternion q, Eigen::Vector3 v, eigen::Vector3 omega)
    // {
    //     uranus_dp::Wrench msg;
    //     msg.force.x = surge  * NORMALIZATION * SCALING_LIN;
    //     msg.force.y = sway   * NORMALIZATION * SCALING_LIN;
    //     msg.force.z = heave  * NORMALIZATION * SCALING_LIN;
    //     msg.torque.x = roll  * NORMALIZATION * SCALING_ANG;
    //     msg.torque.y = pitch * NORMALIZATION * SCALING_ANG;
    //     msg.torque.z = yaw   * NORMALIZATION * SCALING_ANG;
    //     publisher_.publish(msg);
    // }

    // void PublishSetpoint()

    void WaitForMessage()
    {
        while(!message_received_)
        {
            ros::spinOnce();
        }
    }

    static const double EPSILON = 1e-6; // Max absolute error 

 private:
    ros::NodeHandle node_handle_;
    ros::Publisher  state_pub_;
    ros::Publisher  setpoint_pub_;
    ros::Subscriber subscriber_;
    bool message_received_;

    void Callback(const geometry_msgs::Wrench& msg)
    {
        message_received_ = true;
    }

    bool IsNodeReady()
    {
        return (state_pub_.getNumSubscribers() > 0) && (setpoint_pub_.getNumSubscribers() > 0) && (subscriber_.getNumPublishers() > 0);
    }
};

TEST_F(ControllerTest, CheckResponsiveness)
{
    // Publish(1, 2, 3, 4, 5, 5);
    // WaitForMessage();
    ASSERT_TRUE(true);
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
