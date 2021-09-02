#include "hd_key_ctrl.h"

/* Receiving set point from the user */
void initTermios(void){
  tcgetattr(0, &old); /* grab old terminal i/o settings */
  current = old; /* make new settings same as old settings */
  current.c_lflag &= ~ICANON; /* disable buffered i/o */
  current.c_lflag &= ~ECHO; /* set no echo mode */
  tcsetattr(0, TCSANOW, &current); /* use these new terminal i/o settings now */
}
void resetTermios(void){
  tcsetattr(0, TCSANOW, &old);
}
char getch(void){
  char ch;
  initTermios();
  ch = getchar();
  resetTermios();
  return ch;
}

void read_input(void){
	char input;
	
	while(do_not_quit){
		input = getch();
		
		switch(input){
			case 'x': do_not_quit = false;
					  break;
			case 'z': display_help();
					  break;
			
			// Take off and Land
			case 't': calibrate(); //Calibration of height
			          vm.toggle_ready();
                      c++;
			          std::cout << hr << std::endl ;	
			          std::cout << c << std::endl ;			          
					  isPidRunning = true;
					  break; 
			
			// Stop all
			case 's': vm.allStop();
					  for(int i=0;i<3;i++){
						 set_points[i] = 0;
					  }
					  ROS_INFO("Set height: %f",set_points[2]);
					  break;
					  
			// Height (z) +-
			case 'q': set_points[2] += 0.1;
					  ROS_INFO("Set height: %f",set_points[2]);
					  break;
			case 'w': set_points[2] -= 0.1;
					  if(set_points[2] < 0) set_points[2] = 0;
					  ROS_INFO("Set height: %f",set_points[2]);
					  break;
			
			// Setting height
			case 'h': std::cout << "\nEnter height to hover (m): ";
					  std::cin >> set_points[2];
					  break;
			
			// Set height PID gains
			case 'H': 
					  isPidRunning = false;
					  std::cout << "\nEnter height P gain: ";
					  std::cin >> height_controller_.gain_p;
					  std::cout << "Enter height I gain: ";
					  std::cin >> height_controller_.gain_i;
					  std::cout << "Enter height D gain: ";
					  std::cin >> height_controller_.gain_d;
					  height_controller_.reset();
					  isPidRunning = true;
					  break;
			
			// reset height PID
			case 'P': height_controller_.reset();
					  break;
			
			// Yaw +-
			case 'u': set_orient.yaw += 10;
					  #ifdef USE_NEG_ANGLE
						if(set_orient.yaw >= 180.01){
							set_orient.yaw = -(360 - set_orient.yaw);
						}
					  #else
						if(set_orient.yaw >= 360){
							set_orient.yaw = 360 - set_orient.yaw;
						}
					  #endif
					  ROS_INFO("Set yaw: %f",set_orient.yaw);
					  break;
			case 'o': set_orient.yaw -= 10;
					  #ifdef USE_NEG_ANGLE
						  if(set_orient.yaw <= -179.9){
							set_orient.yaw = 180;
						  }
					  #else
						if(set_orient.yaw < 0){
							set_orient.yaw = 360 + set_orient.yaw;
						}
					  #endif
					  ROS_INFO("Set yaw: %f",set_orient.yaw);
					  break;
					  
			// Setting yaw
			case 'y': std::cout << "\nEnter angle (-180,180] (deg): ";
					  std::cin >> set_orient.yaw;
					  break;
					  
			// Set yaw PID gains
			case 'Y': 
					  isPidRunning = false;
					  std::cout << "\nEnter yaw P gain: ";
					  std::cin >> yaw_controller_.gain_p;
					  std::cout << "Enter yaw I gain: ";
					  std::cin >> yaw_controller_.gain_i;
					  std::cout << "Enter yaw D gain: ";
					  std::cin >> yaw_controller_.gain_d;
					  yaw_controller_.reset();
					  isPidRunning = true;
					  break;
		}		
	}
}

void display_help(void){
	std::cout << "\nUse the following commands: "<< std::endl
			<< "t: Take off/Land (cmds work iff human_drone has taken off)" << std::endl
			<< "s: Stop all" << std::endl
			<< "h: Set height (m)" << std::endl
			<< "y: Set yaw angle (deg)" << std::endl
			<< "q: Increase height by 0.1 m" << std::endl
			<< "w: Decrease height by 0.1 m" << std::endl
			<< "u: Counter-clockwise yaw by 10 deg" << std::endl
			<< "o: Clockwise yaw by 10 deg" << std::endl
			<< "H: Set height PID gains" << std::endl
			<< "Y: Set yaw PID gains" << std::endl
			<< "P: Reset height PID" << std::endl
			<< "z: Display help" << std::endl
			<< "x: Quit\n" << std::endl;
}

int main(int argc, char **argv){
	
	ros::init(argc,argv,"viman_semiAuto");
	ros::NodeHandle node;
	
	ros::Subscriber height_subs_ = node.subscribe("/human_drone/height",1,HeightCallbck);
	ros::Subscriber imu_subs_ = node.subscribe("/human_drone/imu",1,ImuCallbck);
	
	vm = Viman(node);
	c = 0;          //Calibration control variable
	height_controller_ = VmPidLinear();
	yaw_controller_ = VmPidRotate();
	
	height_controller_.gain_p = 2;
	height_controller_.gain_i = 0.1;
	height_controller_.gain_d = 0.03;
	
	yaw_controller_.gain_p = 0.6;
	yaw_controller_.gain_i = 0.1;
	yaw_controller_.gain_d = 0.4;
	yaw_controller_.sp_range = 0.3;
	
	display_help();
    if( height_subs_.getTopic() != "")
        ROS_INFO("found altimeter height topic");
    else
        ROS_INFO("cannot find altimeter height topic!");
    if( imu_subs_.getTopic() != "")
        ROS_INFO("found imu topic");
    else
        ROS_INFO("cannot find imu topic!");
        
    isPidRunning = false;
    
	//ros::Rate rate(2);
	
	// begin separate thread to read from the keyboard		
	std::thread reading_input_thread(read_input);
	reading_input_thread.detach();
	
	double prev_time = ros::Time::now().toSec();
	double cur_time;
	double dt;
		
	set_points[2] = 0.5;
	set_orient.yaw = 0;
		
	while(do_not_quit && ros::ok()){
		if(isPidRunning){
			cur_time = ros::Time::now().toSec();
			
			dt = cur_time - prev_time;
			
			if(dt <= 0 ) continue;
			
			cmd_values[2] = height_controller_.update(set_points[2], position[2]-hr, 
									  dt);
									  
			cmd_values[3] = yaw_controller_.update(set_orient.yaw, cur_orient.yaw, dt);
			
			vm.move(cmd_values[0],cmd_values[1],cmd_values[2],cmd_values[3]);
			
			prev_time = cur_time;			
		}
		else{
			prev_time = ros::Time::now().toSec();
		}
		ros::spinOnce();
	}
}
void calibrate(void){
	std::cout << "Calibrating Height..." <<std::endl;
	hr = 0.0;//Variable to store average z-height
	//Take off case
	if(c%2!=0){
	for(int i = 0; i<100; i++)
	{
		hr+=h;
	}
	hr=hr/100.0;
    }
	//Landing
	else{	                                    
    for(int i = 0; i<100; i++)
	{
		hr+=h;
	}
	hr=hr/100.0;
	position[2]+=hr;
 }		
}
void HeightCallbck(const geometry_msgs::PointStamped& height_){
	h = height_.point.z;
	position[2] = height_.point.z;	
}

void ImuCallbck(const sensor_msgs::Imu& imu_){
	cur_orient = get_cardan_angles(imu_.orientation.x, imu_.orientation.y,
								   imu_.orientation.z, imu_.orientation.w);
	
	#ifndef USE_NEG_ANGLE
		// converting to positive angle [0,360)
		if(cur_orient.pitch < 0) cur_orient.pitch += 360;
		if(cur_orient.roll < 0) cur_orient.roll += 360;
		if(cur_orient.yaw < 0) cur_orient.yaw += 360;
	#endif
}
