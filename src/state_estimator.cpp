#include "ros/ros.h"
#include "uranus_dp/State.h"
#include "sensor_msgs/Imu.h"
#include "Eigen/Dense"

// See Fossen 2011, chapter 11.3.3
class ExtendedKalmanFilter
{
public:
	ExtendedKalmanFilter()
	{
		// Constructor goes here

		n_x = 13;
		n_y = 10;
		n_u = 6;
	}

	void sensorCallback(const sensor_msgs::Imu &sensor_msg) // This actually takes a custom message defined in arduino_ros, but not yet
	{
		// Copy message to y_k
		// Call update function maybe (or call it periodically elsewhere)
	}

	void setFrequency(double frequency)
	{
		h = 1/frequency;
	}

	void update()
	{
		// Do everything yo
	}
private:
	ros::NodeHandle n;
	ros::Publisher  stateEstimatePub;
	ros::Subscriber sensorSub;

	// System dimensions
	unsigned int n_x; // Number of states
	unsigned int n_y; // Number of measurements
	unsigned int n_u; // Number of control inputs
	unsigned int n_w; // Length of state noise vector

	// Kalman filter sample time
	double h;

	// State and measurement vectors
	Eigen::VectorXd x_hat; // (n_x) State estimate vector
	Eigen::VectorXd x_bar; // (n_x) State estimate propagation
	Eigen::VectorXd y;     // (n_y) Measurement vector

	// System matrices
	Eigen::MatrixXd B; // (n_x * n_u) Input coefficient matrix
	Eigen::MatrixXd E; // (n_x * n_w) State noise matrix
	Eigen::MatrixXd H; // (n_y * n_x) Measurement matrix

	// Initial values
	Eigen::VectorXd x_0; // (n_x) Initial state vector
	Eigen::MatrixXd P_0; // (n_x * n_x) Initial error covariance matrix

	// Kalman filter matrices
	Eigen::MatrixXd Q;     // (n_w * n_w) Design matrix (symmetric, pos. def.)
	Eigen::MatrixXd R;     // (n_y * n_y) Design matrix (symmetric, pos. def.)
	Eigen::MatrixXd K;     // (n_x * n_y) Kalman gain matrix
	Eigen::MatrixXd P_hat; // (n_x * n_x) Error covariance matrix
	Eigen::MatrixXd P_bar; // (n_x * n_x) Error covariance propagation
	Eigen::VectorXd F;     // (n_x) Some function
	Eigen::MatrixXd Phi;   // (n_x * n_x) Some function
	Eigen::MatrixXd Gamma; // (n_x * n_w) Somu function
}; // End of class ExtendedKalmanFilter

int main(int argc, char **argv)
{
	ros::init(argc, argv, "state_estimator");

	ExtendedKalmanFilter filter;

	double frequency = 100; // Parameter server
	filter.setFrequency(frequency);

	ros::Rate loop_rate(frequency);
	while (ros::ok())
	{
		ros::spinOnce();

		filter.update();

		loop_rate.sleep();
	}
}
