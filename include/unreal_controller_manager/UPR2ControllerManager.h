#include <vector>
#include <ros/ros.h>
#include <pr2_hardware_interface/hardware_interface.h>
#include <pr2_controller_manager/controller_manager.h>
#include "unreal_controller_manager/RegisterRobot.h"
#include <boost/thread/mutex.hpp>

class UControllerManager
{
public:
	UControllerManager();
	virtual ~UControllerManager();

	virtual bool registerRobot(unreal_controller_manager::RegisterRobot::Request &req,
									   unreal_controller_manager::RegisterRobot::Response &res);
	virtual void Init();
	virtual void Run();

private:
	virtual void Update();
	ros::NodeHandle* rosnode_;

	pr2_hardware_interface::HardwareInterface hw_;
	pr2_controller_manager::ControllerManager *cm_;

	pr2_mechanism_model::RobotState *fake_state_;

	void ControllerManagerROSThread();
	boost::thread ros_spinner_thread_;

	std::vector<std::string> JointNames;


};
