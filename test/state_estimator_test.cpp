#include "state_estimator_test.h"

TEST_F(StateEstimatorTest, CheckResponsivenes)
{
    // First message only gives the filter an initial timestamp. From the second
    // message onward the filter can calculate a time difference between input
    // messages, and thus also calculate an output.
    Publish(0,0,0, 0,0,0, 1,0,0,0);
    Publish(0,0,0, 0,0,0, 1,0,0,0);
    WaitForMessage();
}

TEST_F(StateEstimatorTest, CorrectInitialization)
{
    ResetFilter();
    Publish(0,0,0, 0,0,0, 0,0,0,1);
    Publish(0,0,0, 0,0,0, 0,0,0,1);
    WaitForMessage();

    EXPECT_NEAR(p(0), 0, MAX_ERROR);
    EXPECT_NEAR(p(1), 0, MAX_ERROR);
    EXPECT_NEAR(p(2), 0, MAX_ERROR);
    EXPECT_NEAR(q.x(), 0, MAX_ERROR);
    EXPECT_NEAR(q.y(), 0, MAX_ERROR);
    EXPECT_NEAR(q.z(), 0, MAX_ERROR);
    EXPECT_NEAR(q.w(), 1, MAX_ERROR);
    EXPECT_NEAR(v(0), 0, MAX_ERROR);
    EXPECT_NEAR(v(1), 0, MAX_ERROR);
    EXPECT_NEAR(v(2), 0, MAX_ERROR);
    EXPECT_NEAR(w(0), 0, MAX_ERROR);
    EXPECT_NEAR(w(1), 0, MAX_ERROR);
    EXPECT_NEAR(w(2), 0, MAX_ERROR);
}

TEST_F(StateEstimatorTest, ForwardAcceleration)
{
    ResetFilter();
    OneSecondPublish(1,0,0, 0,0,0, 0,0,0,1);
    ros::spinOnce();

    EXPECT_NEAR(p(0), 0.5, MAX_ERROR);
    EXPECT_NEAR(p(1), 0, MAX_ERROR);
    EXPECT_NEAR(p(2), 0, MAX_ERROR);
    EXPECT_NEAR(q.x(), 0, MAX_ERROR);
    EXPECT_NEAR(q.y(), 0, MAX_ERROR);
    EXPECT_NEAR(q.z(), 0, MAX_ERROR);
    EXPECT_NEAR(q.w(), 1, MAX_ERROR);
    EXPECT_NEAR(v(0), 1, MAX_ERROR);
    EXPECT_NEAR(v(1), 0, MAX_ERROR);
    EXPECT_NEAR(v(2), 0, MAX_ERROR);
    EXPECT_NEAR(w(0), 0, MAX_ERROR);
    EXPECT_NEAR(w(1), 0, MAX_ERROR);
    EXPECT_NEAR(w(2), 0, MAX_ERROR);
}

// TEST_F(StateEstimatorTest, Rotate)
// {
//     ResetFilter();
//     // Publish(0,0,STANDARD_GRAVITY,0,0,1); // 1 rad/sec in yaw  (I think)
//     Publish(0,0,0,0,0,1);
//     OneSecondSpin();

//     EXPECT_NEAR(p(0), 0, 0.01);
//     EXPECT_NEAR(p(1), 0, 0.01);
//     EXPECT_NEAR(p(2), 0, 0.01);

//     EXPECT_NEAR(q.w(), 0.877582561890373, 0.01);
//     EXPECT_NEAR(q.x(), 0, 0.01);
//     EXPECT_NEAR(q.y(), 0, 0.01);
//     EXPECT_NEAR(q.z(), 0.479425538604203, 0.01);
// }



StateEstimatorTest::StateEstimatorTest()
{
    pub = nh.advertise<sensor_msgs::Imu>("imu/data", 10);
    sub = nh.subscribe("state_estimate", 10, &StateEstimatorTest::Callback, this);
    client = nh.serviceClient<uranus_dp::ResetStateEstimator>("reset_state_estimator");
    message_received = false;
}

void StateEstimatorTest::SetUp()
{
    while (!IsNodeReady())
        ros::spinOnce();
}

void StateEstimatorTest::Publish(double ax, double ay, double az,
                                 double wx, double wy, double wz,
                                 double qx, double qy, double qz, double qw)
{
    sensor_msgs::Imu msg;
    msg.header.stamp = ros::Time::now();
    msg.linear_acceleration.x = ax;
    msg.linear_acceleration.y = ay;
    msg.linear_acceleration.z = az;
    msg.angular_velocity.x    = wx;
    msg.angular_velocity.y    = wy;
    msg.angular_velocity.z    = wz;
    msg.orientation.x         = qx;
    msg.orientation.y         = qy;
    msg.orientation.z         = qz;
    msg.orientation.w         = qw;
    pub.publish(msg);
}

void StateEstimatorTest::OneSecondPublish(double ax, double ay, double az,
                                          double wx, double wy, double wz,
                                          double qx, double qy, double qz, double qw)
{
    // Set publishing frequency etc.
    int frequency = 5;
    ros::Rate rate(frequency);
    int numPublished = 0;

    // First publish once to initialize
    Publish(0,0,0, 0,0,0, 0,0,0,1);
    ros::spinOnce();
    rate.sleep();

    // If publishing at a frequency of x Hz, then we should loop until we have
    // published x times in order to publish one full second.
    while (numPublished < frequency)
    {
        Publish(ax,ay,az, wx,wy,wz, qx,qy,qz,qw);
        ros::spinOnce();
        rate.sleep();
        // ros::spinOnce();
        numPublished++;
    }
}

void StateEstimatorTest::WaitForMessage()
{
    while (!message_received)
        ros::spinOnce();
}

void StateEstimatorTest::OneSecondSpin()
{
    ros::Time start = ros::Time::now();
    while (ros::Time::now() < start + ros::Duration(1))
        ros::spinOnce();
}

void StateEstimatorTest::ResetFilter()
{
    uranus_dp::ResetStateEstimator srv;
    client.call(srv);
}

bool StateEstimatorTest::GetMessageReceived()
{
    return message_received;
}

void StateEstimatorTest::Callback(const nav_msgs::Odometry& msg)
{
    tf::pointMsgToEigen(msg.pose.pose.position, p);
    tf::quaternionMsgToEigen(msg.pose.pose.orientation, q);
    tf::vectorMsgToEigen(msg.twist.twist.linear, v);
    tf::vectorMsgToEigen(msg.twist.twist.angular, w);
    message_received = true;
}

bool StateEstimatorTest::IsNodeReady()
{
    return (pub.getNumSubscribers() > 0) && (sub.getNumPublishers() > 0);
}



int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "state_estimator_test");

    int ret = RUN_ALL_TESTS();
    ros::shutdown();
    return ret;
}
