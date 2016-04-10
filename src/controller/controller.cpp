#include "ros/ros.h"
#include "quaternion_pd_controller.h"
#include "open_loop_controller.h"
#include "uranus_dp/SetControlMode.h"
#include "joystick/DirectionalInput.h"
#include "../control_mode_enum.h"

class Controller
{
public:
    Controller(unsigned int f)
    {
        frequency = f;
        control_mode = ControlModes::OPEN_LOOP;
        stationkeeper.disable();
        openlooper.enable();
    }

    bool setControlMode(uranus_dp::SetControlMode::Request &req, uranus_dp::SetControlMode::Response &resp)
    {
        control_mode = static_cast<ControlMode>(req.mode);
        switch (control_mode)
        {
        case ControlModes::OPEN_LOOP:
            ROS_INFO("Setting control mode open loop.");
            stationkeeper.disable();
            openlooper.enable();
            break;
        case ControlModes::STATIONKEEPING:
            ROS_INFO("Setting control mode stationkeeping.");
            openlooper.disable();
            stationkeeper.enable();
            break;
        }
        return true;
    }

    void run()
    {
        ros::Rate loop_late(frequency);
        while (ros::ok())
        {
            ros::spinOnce();
            stationkeeper.compute();
        }
    }
private:
    ros::NodeHandle nh;
    unsigned int frequency;

    ControlMode control_mode;
    QuaternionPdController stationkeeper; // I hardly know her.
    OpenLoopController     openlooper;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ROS_INFO("Launching node controller.");
    ros::NodeHandle nh;

    unsigned int frequency = 10;
    Controller controller(frequency);
    ros::ServiceServer ss = nh.advertiseService("set_control_mode", &Controller::setControlMode, &controller);

    controller.run();
    return 0;
}
