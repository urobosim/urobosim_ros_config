#include "unreal_controller_manager/UPR2ControllerManager.h"
#include <string>
#include <math.h>
#include "sensor_msgs/JointState.h"

bool UControllerManager::registerRobot(unreal_controller_manager::RegisterRobot::Request &req,
									   unreal_controller_manager::RegisterRobot::Response &res)
{
	ROS_INFO("Robot Registerd");

	// for(std::string& Joint : req.joint_names)
	// {
	// 	JointNames.push_back(Joint);
	// 	// ROS_INFO("%s",Joint);
	// }
	for(int i =0; i< req.joint_names.size(); i++)
	{
		// ROS_INFO("%s",req.joint_names[i].c_str());
		JointNames.push_back(req.joint_names[i].c_str());
	}

	res.success=true;
	res.status_message = "this is the response";
	return true;
}

UControllerManager::UControllerManager()
{
	if (!ros::isInitialized())
	{
		int argc = 0;
		char** argv = NULL;
		// ros::init(argc,argv,"unreal",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
		ros::init(argc,argv,"unreal",ros::init_options::AnonymousName);
	}
	this->rosnode_ = new ros::NodeHandle("pr2");
}

UControllerManager::~UControllerManager()
{
	this->rosnode_->shutdown();
	this->cm_->~ControllerManager();

	if (this->fake_state_)
	{
		// why does this cause double free corrpution in destruction of RobotState?
		//this->fake_state_->~RobotState();
		delete this->fake_state_;
	}
	this->ros_spinner_thread_.join();
	delete this->rosnode_;
	delete this->cm_;
}

void UControllerManager::Update()
{

}

void UControllerManager::ControllerManagerROSThread()
{
	ROS_INFO_STREAM("Callback thread id=" << boost::this_thread::get_id());

	  //ros::Rate rate(1000);

	  while (this->rosnode_->ok())
	  {
		  //rate.sleep(); // using rosrate gets stuck on model delete
		  usleep(1000);
		  ros::spinOnce();
	  }
}



void UControllerManager::Init()
{

	// pr2_etherCAT calls ros::spin(), we'll thread out one spinner here to mimic that
	this->ros_spinner_thread_ = boost::thread( boost::bind( &UControllerManager::ControllerManagerROSThread,this  )  );

	// Initializes the fake state (for running the transmissions backwards).

	this->cm_ = new pr2_controller_manager::ControllerManager(&hw_,*this->rosnode_);

	//TODO GetSimTime
	this->hw_.current_time_ = ros::Time(0.001);
	if (this->hw_.current_time_ < ros::Time(0.001)) this->hw_.current_time_ == ros::Time(0.001); // hardcoded to minimum of 1ms on start up


	// this->fake_state_ = new pr2_mechanism_model::RobotState(&this->cm_->model_);
}

void UControllerManager::Run()
{
	ros::ServiceServer service = this->rosnode_->advertiseService("RegisterRobot",&UControllerManager::registerRobot, this);

	std::string JointNames[] = {"l_elbow_flex_joint", "l_forearm_roll_joint","l_shoulder_lift_joint", "l_upper_arm_roll_joint"};

	float JointPosition[] = {40,0,0,0};
	float JointVelocity[] = {40,0,0,0};
	float JointEffort[] = {4000000,0,0,0};

	int MsgEleNum = 4;
	sensor_msgs::JointState msg;
	msg.name.resize(MsgEleNum);
	msg.position.resize(MsgEleNum);
	msg.velocity.resize(MsgEleNum);
	msg.effort.resize(MsgEleNum);

	ROS_INFO("starting gazebo_ros_controller_manager plugin in ns: %s","pr2");

	ros::Publisher JointStatePublisher = this->rosnode_->advertise<sensor_msgs::JointState>("TargetJointState", 5000);


	int tick = 0;
	while (this->rosnode_->ok())
	{
		// rate.sleep(); // using rosrate gets stuck on model delete

		for(int i = 0; i < MsgEleNum; i++)
		{
			msg.name[i] = JointNames[i];
			msg.position[i] = JointPosition[i] + JointPosition[i]*sin(tick*0.01);
			msg.velocity[i] = JointVelocity[i]*sin(tick*0.01);
			msg.effort[i] = JointEffort[i]*sin(tick*0.01);
		}

		tick++;

		JointStatePublisher.publish(msg);
		// ROS_INFO("publish");

		usleep(10000);
		ros::spinOnce();
	}
}

int main(int argc, char **argv)
{
	UControllerManager* Manager = new UControllerManager();
	Manager->Init();
	Manager->Run();

	// delete Manager;
}
