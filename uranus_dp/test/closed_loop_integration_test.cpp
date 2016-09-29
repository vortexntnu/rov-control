#include "ros/ros.h"
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "sensor_msgs/Imu.h"
#include "maelstrom_msgs/JoystickMotionCommand.h"
#include "maelstrom_msgs/ThrusterForces.h"
#include "../src/control_mode_enum.h"
#include "../src/eigen_typedefs.h"

class ClosedLoopIntegrationTest : public ::testing::Test
{
public:
    ClosedLoopIntegrationTest()
    {
        joyPub = nh.advertise<maelstrom_msgs::JoystickMotionCommand>("joystick_motion_command", 10);
        imuPub = nh.advertise<sensor_msgs::Imu>("sensor_raw", 10);
        sub = nh.subscribe("thruster_forces", 10, &ClosedLoopIntegrationTest::Callback, this);
        message_received = false;
    }

    void SetUp()
    {
        while (!IsNodeReady())
            ros::spinOnce();
    }

    void JoystickPublish(double forward, double right, double down, double tilt_up, double turn_right, ControlMode control_mode)
    {
        maelstrom_msgs::JoystickMotionCommand msg;
        msg.forward    = forward;
        msg.right      = right;
        msg.down       = down;
        msg.tilt_up    = tilt_up;
        msg.turn_right = turn_right;
        msg.control_mode = static_cast<int>(control_mode);
        joyPub.publish(msg);
    }

    void SensorPublish(double ax, double ay, double az,
                      double wx, double wy, double wz)
    {
        sensor_msgs::Imu msg;
        msg.linear_acceleration.x  = ax;
        msg.linear_acceleration.y  = ay;
        msg.linear_acceleration.z  = az;
        msg.angular_velocity.x = wx;
        msg.angular_velocity.y = wy;
        msg.angular_velocity.z = wz;
        imuPub.publish(msg);
    }

    void WaitForMessage()
    {
        message_received = false;
        while (!message_received)
            ros::spinOnce();
    }

    void OneSecondSpin()
    {
        ros::Time start = ros::Time::now();
        while (ros::Time::now() < start + ros::Duration(1))
            ros::spinOnce();
    }

    double F_A;
    double F_B;
    double F_C;
    double F_D;
    double F_E;
    double F_F;

    static const double MAX_ERROR = 1e-3;

private:
    ros::NodeHandle nh;
    ros::Publisher  joyPub;
    ros::Publisher  imuPub;
    ros::Subscriber sub;
    bool message_received;

    void Callback(const maelstrom_msgs::ThrusterForces& msg)
    {
        F_A = msg.F1;
        F_B = msg.F2;
        F_C = msg.F3;
        F_D = msg.F4;
        F_E = msg.F5;
        F_F = msg.F6;
        message_received = true;
    }

    bool IsNodeReady()
    {
        return (joyPub.getNumSubscribers() > 0) && (imuPub.getNumSubscribers() > 0) && (sub.getNumPublishers() > 0);
    }
};

TEST_F(ClosedLoopIntegrationTest, CheckResponsiveness)
{
    JoystickPublish(0,0,0,0,0,ControlModes::POSITION_HOLD);
    SensorPublish(0,0,9.80665,0,0,0);
    WaitForMessage();
}

TEST_F(ClosedLoopIntegrationTest, ZeroInput)
{
    JoystickPublish(0,0,0,0,0,ControlModes::POSITION_HOLD);
    SensorPublish(0,0,9.80665,0,0,0);
    WaitForMessage();

    EXPECT_NEAR(F_A, 0, MAX_ERROR);
    EXPECT_NEAR(F_B, 0, MAX_ERROR);
    EXPECT_NEAR(F_C, 0, MAX_ERROR);
    EXPECT_NEAR(F_D, 0, MAX_ERROR);
    EXPECT_NEAR(F_E, 0, MAX_ERROR);
    EXPECT_NEAR(F_F, 0, MAX_ERROR);
}

TEST_F(ClosedLoopIntegrationTest, SurgePositionError)
{
    JoystickPublish(1,0,0,0,0,ControlModes::POSITION_HOLD);
    SensorPublish(0,0,9.80665,0,0,0);
    WaitForMessage();

    EXPECT_TRUE(F_A < 0);
    EXPECT_TRUE(F_B > 0);
    EXPECT_TRUE(F_C < 0);
    EXPECT_TRUE(F_D > 0);
    EXPECT_TRUE(F_E > 0);
    EXPECT_TRUE(F_F < 0);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "closed_loop_integration_test");

    int ret = RUN_ALL_TESTS();
    ros::shutdown();
    return ret;
}
