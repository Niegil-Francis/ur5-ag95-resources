// UI
#include "gui.h"

// ROS
#include <ros/ros.h>

// Message
#include <std_msgs/Int8.h>
#include <trajectory_msgs/JointTrajectory.h>

// Misc
#include <signal.h>

#define SHOULDER_PAN_JNT 	0
#define SHOULDER_LIFT_JNT 	1
#define ELBOW_JNT	 		2
#define WRIST_1_JNT 		3
#define WRIST_2_JNT 		4
#define WRIST_3_JNT 		5

// Trajectory types
#define STOP 			0
#define HOME 			1
#define START_POSE 		2
#define THIRD_ORDER_SCALE 	3
#define SCALE_VEL 		4

#define SIM				// comment for hardware control

#ifdef SIM
	#define CONTROL_TOPIC "/pos_joint_traj_controller/command"
#else
	#define CONTROL_TOPIC "/scaled_pos_joint_traj_controller/command"
#endif

class TrajReplaceGui : public QMainWindow{

private:
	/// @brief Instance UI
	Ui::mainWindow ui;

	// ROS related variables >>>>>>>>>>>>>>

	/// @brief ROS node handle
	ros::NodeHandle nh_;

	/// @brief Execute command publisher
	ros::Publisher exe_pub_;

	/// @brief Trajectory publisher
	ros::Publisher traj_pub_;

	/// @brief Trajectory message
	trajectory_msgs::JointTrajectory traj_msg_;

	/// @brief Execute message instance
	std_msgs::Int8 exe_msg_;

public:
	TrajReplaceGui(QMainWindow *parent = nullptr) : QMainWindow(parent){
		// Setting up communication (pub-sub)
		nh_ = ros::NodeHandle();
		exe_pub_ = nh_.advertise<std_msgs::Int8>("traj_type", 1);
		traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(CONTROL_TOPIC, 1);

		// populating messages with default values
		traj_msg_.points.resize(1);
		traj_msg_.points[0].velocities.resize(6);
		traj_msg_.points[0].accelerations.resize(6);
		traj_msg_.points[0].positions.resize(6);
		traj_msg_.joint_names.resize(6);
		traj_msg_.joint_names[SHOULDER_PAN_JNT] = "shoulder_pan_joint";
		traj_msg_.joint_names[SHOULDER_LIFT_JNT] = "shoulder_lift_joint";
		traj_msg_.joint_names[ELBOW_JNT] = "elbow_joint";
		traj_msg_.joint_names[WRIST_1_JNT] = "wrist_1_joint";
		traj_msg_.joint_names[WRIST_2_JNT] = "wrist_2_joint";
		traj_msg_.joint_names[WRIST_3_JNT] = "wrist_3_joint";

		exe_msg_.data = STOP;

		// setup the UI
		ui.setupUi(this);

		connect(ui.exeBtn, &QPushButton::clicked, this, &TrajReplaceGui::exeBtnClicked);
		connect(ui.homeRbtn, &QRadioButton::clicked, this, &TrajReplaceGui::homeBtnClicked);
		connect(ui.startPoseRbtn, &QRadioButton::clicked, this, &TrajReplaceGui::startPoseBtnClicked);
		connect(ui.thirdOrderScaleRbtn, &QRadioButton::clicked, this, &TrajReplaceGui::replaceTrajBtnClicked);
		connect(ui.scaleVelRBtn, &QRadioButton::clicked, this, &TrajReplaceGui::scaleVelBtnClicked);
	}

private slots:

	void exeBtnClicked(){
		static bool is_exe_msg_sent = false;

		if (is_exe_msg_sent){
			exe_msg_.data = STOP;
			ui.exeBtn->setText("Execute");

			is_exe_msg_sent = false;
		}
		else{
			switch(exe_msg_.data){
				case HOME:	traj_msg_.points[0].positions = std::vector<double>({0, -1.5708559195147913, 0, -1.5707481543170374, 0, 0});
							traj_msg_.points[0].time_from_start = ros::Duration(2);
							traj_msg_.header.stamp = ros::Time::now();

							traj_pub_.publish(traj_msg_);
							break;
				case START_POSE: traj_msg_.points[0].positions = std::vector<double>({0.7854925394058228, -2.0945685545550745, 2.094480037689209,
											-1.5707839171039026, -1.5714247862445276, -0.00031119981874638825});
								traj_msg_.points[0].time_from_start = ros::Duration(2);
								traj_msg_.header.stamp = ros::Time::now();

								traj_pub_.publish(traj_msg_);
								break;
				default: ui.exeBtn->setText("Stop");
						 is_exe_msg_sent = true;
			}
			
			ui.homeRbtn->setAutoExclusive(false);
			ui.startPoseRbtn->setAutoExclusive(false);
			ui.thirdOrderScaleRbtn->setAutoExclusive(false);
			ui.scaleVelRBtn->setAutoExclusive(false);

			ui.homeRbtn->setChecked(false);
			ui.startPoseRbtn->setChecked(false);
			ui.thirdOrderScaleRbtn->setChecked(false);
			ui.scaleVelRBtn->setChecked(false);
			
			ui.homeRbtn->setAutoExclusive(true);
			ui.startPoseRbtn->setAutoExclusive(true);
			ui.thirdOrderScaleRbtn->setAutoExclusive(true);
			ui.scaleVelRBtn->setAutoExclusive(true);
		}

		exe_pub_.publish(exe_msg_);
	}

	void homeBtnClicked(){
		exe_msg_.data = HOME;
	}

	void startPoseBtnClicked(){
		exe_msg_.data = START_POSE;
	}

	void replaceTrajBtnClicked(){
		exe_msg_.data = THIRD_ORDER_SCALE;
	}

	void scaleVelBtnClicked(){
		exe_msg_.data = SCALE_VEL;
	}

};

void signalCallbackHandler(int signum){
	ROS_INFO_STREAM("Shutting down node!");
	QApplication::quit();
	ros::shutdown();
}

int main(int argc, char* argv[]){
	// setting up ROS
	ros::init(argc, argv, "traj_replace_gui", ros::init_options::NoSigintHandler);
	ros::AsyncSpinner spinner(0);
	spinner.start();	

	signal(SIGINT, signalCallbackHandler);

	QApplication a(argc, argv);
	TrajReplaceGui gui;
	gui.show();

	return a.exec();
}