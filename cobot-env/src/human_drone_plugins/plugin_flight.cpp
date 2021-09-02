#include "plugin_flight.h"


#include <cmath>
#include <stdlib.h>
#include <iostream>


namespace gazebo {

// Constructor
VimanFlight::VimanFlight(){ 
  navi_state = LANDED_MODEL;	// Initial navigation state
  m_posCtrl = false;			// Position control mode off
  m_velMode = false;			// Velocity control mode off
}

// Destructor
VimanFlight::~VimanFlight(){
  node_handle_->shutdown();
  delete node_handle_;
}

// Load the plugin
void VimanFlight::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){

  if(!ros::isInitialized()){
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_init.so' in the gazebo_ros package");
  }
  
  world = _model->GetWorld();
  ROS_INFO("VIMAN Control plugin is loading!");
  
  //load parameters
  cmd_normal_topic_ = "/cmd_vel";
  takeoff_topic_ = "/human_drone/takeoff";
  land_topic_ = "/human_drone/land";
  reset_topic_ = "/human_drone/reset";
  posctrl_topic_ = "/human_drone/posctrl";
  gt_topic_ = "/human_drone/gt_pose";
  switch_mode_topic_ = "/human_drone/vel_mode";
  
  // Obtaining parameters from SDF
  if (!_sdf->HasElement("imuTopic"))
    imu_topic_.clear();
  else
    imu_topic_ = _sdf->GetElement("imuTopic")->Get<std::string>();
  
  // link here corresponds to body/base link of the drone
  if (!_sdf->HasElement("bodyName")){
    link = _model->GetLink();
    link_name_ = link->GetName();
  }
  else {
    link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
    link = boost::dynamic_pointer_cast<physics::Link>(world->EntityByName(link_name_));
  }

  if (!link){
    ROS_FATAL("gazebo_ros_baro plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }

  if (!_sdf->HasElement("maxForce"))
    max_force_ = -1;
  else
    max_force_ = _sdf->GetElement("maxForce")->Get<double>();


  if (!_sdf->HasElement("motionSmallNoise"))
    motion_small_noise_ = 0;
  else
    motion_small_noise_ = _sdf->GetElement("motionSmallNoise")->Get<double>();

  if (!_sdf->HasElement("motionDriftNoise"))
    motion_drift_noise_ = 0;
  else
    motion_drift_noise_ = _sdf->GetElement("motionDriftNoise")->Get<double>();

  if (!_sdf->HasElement("motionDriftNoiseTime"))
    motion_drift_noise_time_ = 1.0;
  else
    motion_drift_noise_time_ = _sdf->GetElement("motionDriftNoiseTime")->Get<double>();


  // get inertia and mass of quadrotor body
  inertia = link->GetInertial()->PrincipalMoments();
  mass = link->GetInertial()->Mass();

  node_handle_ = new ros::NodeHandle;
  

  // subscribe command: control command
  if (!cmd_normal_topic_.empty()){
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::Twist>(
      cmd_normal_topic_, 1,
      boost::bind(&VimanFlight::CmdCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    cmd_subscriber_ = node_handle_->subscribe(ops);
    
    if( cmd_subscriber_.getTopic() != "")
        ROS_INFO_NAMED("viman_controller", "Using cmd_topic %s.", cmd_normal_topic_.c_str());
    else
        ROS_INFO("cannot find the command topic!");
  }
  
  // subscribe command: position control command
  if (!posctrl_topic_.empty()){
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Bool>(
      posctrl_topic_, 1,
      boost::bind(&VimanFlight::PosCtrlCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    posctrl_subscriber_ = node_handle_->subscribe(ops);
    
    if( posctrl_subscriber_.getTopic() != "")
        ROS_INFO("find the position control topic!");
    else
        ROS_INFO("cannot find the position control topic!");
  }
  
  /* // subscribe to imu
  if (!imu_topic_.empty()){
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<sensor_msgs::Imu>(
      imu_topic_, 1,
      boost::bind(&VimanFlight::ImuCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    imu_subscriber_ = node_handle_->subscribe(ops);
    
    if(imu_subscriber_.getTopic() !="")
        ROS_INFO_NAMED("quadrotor_simple_controller", "Using imu information on topic %s as source of orientation and angular velocity.", imu_topic_.c_str());
    else
        ROS_INFO("cannot find the IMU topic!");
  }
  * */

  // subscribe command: take off command
  if (!takeoff_topic_.empty()){
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Empty>(
      takeoff_topic_, 1,
      boost::bind(&VimanFlight::TakeoffCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    takeoff_subscriber_ = node_handle_->subscribe(ops);
    if( takeoff_subscriber_.getTopic() != "")
        ROS_INFO("find the takeoff topic");
    else
        ROS_INFO("cannot find the takeoff topic!");
  }

  // subscribe command: land command
  if (!land_topic_.empty()){
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Empty>(
      land_topic_, 1,
      boost::bind(&VimanFlight::LandCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    land_subscriber_ = node_handle_->subscribe(ops);
  }

  // subscribe command: reset command
  if (!reset_topic_.empty()){
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Empty>(
      reset_topic_, 1,
      boost::bind(&VimanFlight::ResetCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    reset_subscriber_ = node_handle_->subscribe(ops);
  }
  
  // subscribe command: switch control mode
  if (!switch_mode_topic_.empty()){
      ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Bool>(
        switch_mode_topic_, 1,
        boost::bind(&VimanFlight::SwitchModeCallback, this, _1),
        ros::VoidPtr(), &callback_queue_);
      switch_mode_subscriber_ = node_handle_->subscribe(ops);
  }
  
  // publishers
  if (!gt_topic_.empty()){
      pub_gt_pose_ = node_handle_->advertise<geometry_msgs::Pose>("human_drone/gt_pose",1024);    
  }
  
  pub_gt_vec_ = node_handle_->advertise<geometry_msgs::Twist>("human_drone/gt_vel", 1024);
  pub_gt_acc_ = node_handle_->advertise<geometry_msgs::Twist>("human_drone/gt_acc", 1024);
      
  LoadControllerSettings(_model, _sdf);
  
  Reset();

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&VimanFlight::Update, this));
}

void VimanFlight::LoadControllerSettings(physics::ModelPtr _model, sdf::ElementPtr _sdf){
    controllers_.roll.Load(_sdf, "rollpitch");
    controllers_.pitch.Load(_sdf, "rollpitch");
    controllers_.yaw.Load(_sdf, "yaw");
    controllers_.velocity_x.Load(_sdf, "velocityXY");
    controllers_.velocity_y.Load(_sdf, "velocityXY");
    controllers_.velocity_z.Load(_sdf, "velocityZ");
    
    controllers_.pos_x.Load(_sdf, "positionXY");
    controllers_.pos_y.Load(_sdf, "positionXY");
    controllers_.pos_z.Load(_sdf, "positionZ");
    
}

// Callbacks
void VimanFlight::CmdCallback(const geometry_msgs::TwistConstPtr& cmd){
  cmd_val = *cmd;

  static common::Time last_sim_time = world->SimTime();
  static double time_counter_for_drift_noise = 0;
  static double drift_noise[4] = {0.0, 0.0, 0.0, 0.0};
  // Get simulator time
  common::Time cur_sim_time = world->SimTime();
  double dt = (cur_sim_time - last_sim_time).Double();
  // save last time stamp
  last_sim_time = cur_sim_time;

  // generate noise
  if(time_counter_for_drift_noise > motion_drift_noise_time_){
    drift_noise[0] = 2*motion_drift_noise_*(drand48()-0.5);
    drift_noise[1] = 2*motion_drift_noise_*(drand48()-0.5);
    drift_noise[2] = 2*motion_drift_noise_*(drand48()-0.5);
    drift_noise[3] = 2*motion_drift_noise_*(drand48()-0.5);
    time_counter_for_drift_noise = 0.0;
  }
  time_counter_for_drift_noise += dt;

  cmd_val.angular.x += drift_noise[0] + 2*motion_small_noise_*(drand48()-0.5);
  cmd_val.angular.y += drift_noise[1] + 2*motion_small_noise_*(drand48()-0.5);
  cmd_val.angular.z += drift_noise[3] + 2*motion_small_noise_*(drand48()-0.5);
  cmd_val.linear.z += drift_noise[2] + 2*motion_small_noise_*(drand48()-0.5);

}

void VimanFlight::PosCtrlCallback(const std_msgs::BoolConstPtr& cmd){
    m_posCtrl = cmd->data;
}

/*
void VimanFlight::ImuCallback(const sensor_msgs::ImuConstPtr& imu){
  //directly read the quternion from the IMU data
  pose.Rot().Set(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
  euler = pose.Rot().Euler();
  angular_velocity = pose.Rot().RotateVector(ignition::math::Vector3d(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z));
}
* */

void VimanFlight::TakeoffCallback(const std_msgs::EmptyConstPtr& msg){
  if(navi_state == LANDED_MODEL)
  {
    navi_state = TAKEOFF_READY;
    m_timeAfterCmd = 0;
    ROS_INFO("Ready to take off!!");
  }
}

void VimanFlight::LandCallback(const std_msgs::EmptyConstPtr& msg){
  if(navi_state == FLYING_MODEL||navi_state == TAKEOFF_READY)
  {
    navi_state = LANDING_MODEL;
    m_timeAfterCmd = 0;
    ROS_INFO("Viman lands.");
  }

}

void VimanFlight::ResetCallback(const std_msgs::EmptyConstPtr& msg){
  ROS_INFO("Reset Viman");
}

void VimanFlight::SwitchModeCallback(const std_msgs::BoolConstPtr& msg){
    m_velMode = msg->data;
}

// Update the controller
void VimanFlight::Update(){
    
    // Get new commands/state
    callback_queue_.callAvailable();
  
    // Get simulator time
    common::Time sim_time = world->SimTime();
    double dt = (sim_time - last_time).Double();
    if (dt == 0.0) return;
    
    UpdateState(dt);
    UpdateDynamics(dt);
    
    // save last time stamp
    last_time = sim_time;   
}

void VimanFlight::UpdateState(double dt){
    if(navi_state == TAKEOFF_READY){
        m_timeAfterCmd += dt;
        if(m_timeAfterCmd > 0.5){
            navi_state = FLYING_MODEL;
        }
    }
    else if(navi_state == LANDING_MODEL){
        m_timeAfterCmd += dt;
        if(m_timeAfterCmd > 1.0){
            navi_state = LANDED_MODEL;
        }
    }
    else
        m_timeAfterCmd = 0;
}


void VimanFlight::UpdateDynamics(double dt){
    ignition::math::Vector3d force, torque;
   
    // Get Pose/Orientation from Gazebo (if no state subscriber is active)
   if (imu_subscriber_.getTopic()==""){
      pose = link->WorldPose();
      angular_velocity = link->WorldAngularVel();
      euler = pose.Rot().Euler();
      acceleration = (link->WorldLinearVel() - velocity) / dt;
      velocity = link->WorldLinearVel();
    }
    
    
    //publish the ground truth pose of the drone to the ROS topic
    geometry_msgs::Pose gt_pose;
    gt_pose.position.x = pose.Pos().X();
    gt_pose.position.y = pose.Pos().Y();
    gt_pose.position.z = pose.Pos().Z();
    
    gt_pose.orientation.w = pose.Rot().W();
    gt_pose.orientation.x = pose.Rot().X();
    gt_pose.orientation.y = pose.Rot().Y();
    gt_pose.orientation.z = pose.Rot().Z();
    pub_gt_pose_.publish(gt_pose);
    
    //convert the acceleration and velocity into the body frame
    ignition::math::Vector3d body_vel = pose.Rot().RotateVector(velocity);
    ignition::math::Vector3d body_acc = pose.Rot().RotateVector(acceleration);
    
    //publish the velocity
    geometry_msgs::Twist tw;
    tw.linear.x = body_vel.X();
    tw.linear.y = body_vel.Y();
    tw.linear.z = body_vel.Z();
    pub_gt_vec_.publish(tw);
    
    //publish the acceleration
    tw.linear.x = body_acc.X();
    tw.linear.y = body_acc.Y();
    tw.linear.z = body_acc.Z();
    pub_gt_acc_.publish(tw);
    
            
    ignition::math::Vector3d poschange = pose.Pos() - position;
    position = pose.Pos();
    
  
    // Get gravity
    ignition::math::Vector3d gravity_body = pose.Rot().RotateVector(world->Gravity());
    double gravity = gravity_body.Length();
    double load_factor = gravity * gravity / world->Gravity().Dot(gravity_body);  // Get gravity
  
    // Rotate vectors to coordinate frames relevant for control
    ignition::math::Quaterniond heading_quaternion(cos(euler.Z()/2),0,0,sin(euler.Z()/2));
    ignition::math::Vector3d velocity_xy = heading_quaternion.RotateVectorReverse(velocity);
    ignition::math::Vector3d acceleration_xy = heading_quaternion.RotateVectorReverse(acceleration);
    ignition::math::Vector3d angular_velocity_body = pose.Rot().RotateVectorReverse(angular_velocity);
  
    // update controllers
    force.Set(0.0, 0.0, 0.0);
    torque.Set(0.0, 0.0, 0.0);
    
    if(m_posCtrl){
        //position control : Maneuvring via X, Y, Z coordinates
        if(navi_state == FLYING_MODEL){
            double vx = controllers_.pos_x.update(cmd_val.linear.x, position.X(), poschange.X(), dt);
            double vy = controllers_.pos_y.update(cmd_val.linear.y, position.Y(), poschange.Y(), dt);
            double vz = controllers_.pos_z.update(cmd_val.linear.z, position.Z(), poschange.Z(), dt);

            ignition::math::Vector3d vb = heading_quaternion.RotateVectorReverse(ignition::math::Vector3d(vx,vy,vz));
            
            double pitch_command =  controllers_.velocity_x.update(vb.X(), velocity_xy.X(), acceleration_xy.X(), dt) / gravity;
            double roll_command  = -controllers_.velocity_y.update(vb.Y(), velocity_xy.Y(), acceleration_xy.Y(), dt) / gravity;
            torque.X() = inertia.X() *  controllers_.roll.update(roll_command, euler.X(), angular_velocity_body.X(), dt);
            torque.Y() = inertia.Y() *  controllers_.pitch.update(pitch_command, euler.Y(), angular_velocity_body.Y(), dt);            
            force.Z()  = mass      * (controllers_.velocity_z.update(vz,velocity.Z(), acceleration.Z(), dt) + load_factor * gravity);
        }
    }
    else{
        //normal control
        if(navi_state == FLYING_MODEL){
          //hovering
          double pitch_command =  controllers_.velocity_x.update(cmd_val.linear.x, velocity_xy.X(), acceleration_xy.X(), dt) / gravity;
          double roll_command  = -controllers_.velocity_y.update(cmd_val.linear.y, velocity_xy.Y(), acceleration_xy.Y(), dt) / gravity;
          torque.X() = inertia.X()*controllers_.roll.update(roll_command, euler.X(), angular_velocity_body.X(), dt);
          torque.Y() = inertia.Y()*controllers_.pitch.update(pitch_command, euler.Y(), angular_velocity_body.Y(), dt);
        }
        else{
          //control by velocity
          if(m_velMode){
              double pitch_command =  controllers_.velocity_x.update(cmd_val.angular.x, velocity_xy.X(), velocity_xy.X(), dt) / gravity;
              double roll_command  = -controllers_.velocity_y.update(cmd_val.angular.y, velocity_xy.Y(), velocity_xy.Y(), dt) / gravity;
              torque.X() = inertia.X()*controllers_.roll.update(roll_command, euler.X(), angular_velocity_body.X(), dt);
              torque.Y() = inertia.Y()*controllers_.pitch.update(pitch_command, euler.Y(), angular_velocity_body.Y(), dt);              
          }
          else{
            //control by tilting
            torque.X() = inertia.X()*controllers_.roll.update(cmd_val.angular.x, euler.X(), angular_velocity_body.X(), dt);
            torque.Y() = inertia.Y()*controllers_.pitch.update(cmd_val.angular.y, euler.Y(), angular_velocity_body.Y(), dt);
          }
        }
        torque.Z() = inertia.Z()*controllers_.yaw.update(cmd_val.angular.z, angular_velocity.Z(), 0, dt);
        force.Z()  = mass*(controllers_.velocity_z.update(cmd_val.linear.z,  velocity.Z(), acceleration.Z(), dt) + load_factor * gravity);
    }

    if (max_force_ > 0.0 && force.Z() > max_force_) force.Z() = max_force_;
    if (force.Z() < 0.0) force.Z() = 0.0;
    
    // process robot state information
	if(navi_state == FLYING_MODEL){
      link->AddRelativeForce(force);
      link->AddRelativeTorque(torque);
    }
    else if(navi_state == TAKEOFF_READY){
	  link->AddRelativeForce(force*0);
      link->AddRelativeTorque(force*0);
	}
    else if(navi_state == LANDING_MODEL){
      link->AddRelativeForce(force*0.8);
      link->AddRelativeTorque(torque*0.8);
    }

}

// Reset the controller
void VimanFlight::Reset(){
  controllers_.roll.reset();
  controllers_.pitch.reset();
  controllers_.yaw.reset();
  controllers_.velocity_x.reset();
  controllers_.velocity_y.reset();
  controllers_.velocity_z.reset();

  link->SetForce(ignition::math::Vector3d(0,0,0));
  link->SetTorque(ignition::math::Vector3d(0,0,0));

  // reset state
  pose.Reset();
  velocity.Set();
  angular_velocity.Set();
  acceleration.Set();
  euler.Set();
  state_stamp = ros::Time();
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(VimanFlight)

} // namespace gazebo
