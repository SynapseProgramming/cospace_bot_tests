//time test program to enable the use of a custom delay function.
#define CsBot_AI_H//DO NOT delete this line
#ifndef CSBOT_REAL
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <windows.h>
#include <stdbool.h>
#include <time.h>
#define DLL_EXPORT extern __declspec(dllexport)
#define false 0
#define true 1
#endif//The robot ID : It must be two char, such as '00','kl' or 'Cr'.
char AI_MyID[2] = {'0','2'};

const double pi = 3.141592654;

int step=0;
int robot_heading=0;

//when set to true, current poses would be set as initial coordinates for odom.
bool init_odometry=false;

//previous x and y poses
int previous_x_pose=0;
int previous_y_pose=0;

//reliable poses.
double pose_x=0;
double pose_y=0;

//pose estimates by odometry.
double estimated_x=0;
double estimated_y=0;

int Duration = 0;
int SuperDuration = 0;
int bGameEnd = false;
int CurAction = -1;
int CurGame = 0;
int SuperObj_Num = 0;
int SuperObj_X = 0;
int SuperObj_Y = 0;
int Teleport = 0;
int LoadedObjects = 0;
int US_Front = 0;
int US_Left = 0;
int US_Right = 0;
int CSLeft_R = 0;
int CSLeft_G = 0;
int CSLeft_B = 0;
int CSRight_R = 0;
int CSRight_G = 0;
int CSRight_B = 0;
int PositionX = 0;
int PositionY = 0;
int TM_State = 0;
int Compass = 0;
int Time = 0;
int WheelLeft = 0;
int WheelRight = 0;
int LED_1 = 0;
int MyState = 0;
int AI_SensorNum = 13;

// function prototypes declared here

//delay function. returns true if specified time has been reached.
bool delay(int milli_seconds);
//helper function for conversion to cs wheelspeeds aka motor units
int mu_conversion(double wheel_velocity);
//function which calculates angular shaft speeds in deg/s given linear V(mm/s)and angular W(deg/s)
void compute_cospace_velocities(double linear, double angular);
//function which moves bot to specified pose. returns true if the bot has reached the goal. false if its still moving.
bool go_to_goal(double goal_x, double goal_y,double robot_x,double robot_y,double robot_heading);
// function returns the linear distance travelled by each wheel given its motor units, after 61.22 ms.
double wheel_displacements(int motor_units);
//pseudo odom.
void pseudo_odometry(double current_heading);
//function which enables a stable pose to be read. stable poses are pose_x and pose_y
void compute_reliable_pose();

#define CsBot_AI_C//DO NOT delete this line



DLL_EXPORT void SetGameID(int GameID)
{
    CurGame = GameID;
    bGameEnd = 0;
}

DLL_EXPORT int GetGameID()
{
    return CurGame;
}

//Only Used by CsBot Dance Platform
DLL_EXPORT int IsGameEnd()
{
    return bGameEnd;
}

#ifndef CSBOT_REAL

DLL_EXPORT char* GetDebugInfo()
{
    char info[1024];
    sprintf(info, "Duration=%d;SuperDuration=%d;bGameEnd=%d;CurAction=%d;CurGame=%d;SuperObj_Num=%d;SuperObj_X=%d;SuperObj_Y=%d;Teleport=%d;LoadedObjects=%d;US_Front=%d;US_Left=%d;US_Right=%d;CSLeft_R=%d;CSLeft_G=%d;CSLeft_B=%d;CSRight_R=%d;CSRight_G=%d;CSRight_B=%d;PositionX=%d;PositionY=%d;TM_State=%d;Compass=%d;Time=%d;WheelLeft=%d;WheelRight=%d;LED_1=%d;MyState=%d;",Duration,SuperDuration,bGameEnd,CurAction,CurGame,SuperObj_Num,SuperObj_X,SuperObj_Y,Teleport,LoadedObjects,US_Front,US_Left,US_Right,CSLeft_R,CSLeft_G,CSLeft_B,CSRight_R,CSRight_G,CSRight_B,PositionX,PositionY,TM_State,Compass,Time,WheelLeft,WheelRight,LED_1,MyState);
    return info;
}

DLL_EXPORT char* GetTeamName()
{
     return "CoSpace Team";
}

DLL_EXPORT int GetCurAction()
{
    return CurAction;
}

//Only Used by CsBot Rescue Platform
DLL_EXPORT int GetTeleport()
{
    return Teleport;
}

//Only Used by CsBot Rescue Platform
DLL_EXPORT void SetSuperObj(int X, int Y, int num)
{
    SuperObj_X = X;
    SuperObj_Y = Y;
    SuperObj_Num = num;
}
//Only Used by CsBot Rescue Platform
DLL_EXPORT void GetSuperObj(int *X, int *Y, int *num)
{
    *X = SuperObj_X;
    *Y = SuperObj_Y;
    *num = SuperObj_Num;
}

#endif ////CSBOT_REAL

DLL_EXPORT void SetDataAI(volatile int* packet, volatile int *AI_IN)
{

    int sum = 0;

    US_Front = AI_IN[0]; packet[0] = US_Front; sum += US_Front;
    US_Left = AI_IN[1]; packet[1] = US_Left; sum += US_Left;
    US_Right = AI_IN[2]; packet[2] = US_Right; sum += US_Right;
    CSLeft_R = AI_IN[3]; packet[3] = CSLeft_R; sum += CSLeft_R;
    CSLeft_G = AI_IN[4]; packet[4] = CSLeft_G; sum += CSLeft_G;
    CSLeft_B = AI_IN[5]; packet[5] = CSLeft_B; sum += CSLeft_B;
    CSRight_R = AI_IN[6]; packet[6] = CSRight_R; sum += CSRight_R;
    CSRight_G = AI_IN[7]; packet[7] = CSRight_G; sum += CSRight_G;
    CSRight_B = AI_IN[8]; packet[8] = CSRight_B; sum += CSRight_B;
    PositionX = AI_IN[9]; packet[9] = PositionX; sum += PositionX;
    PositionY = AI_IN[10]; packet[10] = PositionY; sum += PositionY;
    TM_State = AI_IN[11]; packet[11] = TM_State; sum += TM_State;
    Compass = AI_IN[12]; packet[12] = Compass; sum += Compass;
    Time = AI_IN[13]; packet[13] = Time; sum += Time;
    packet[14] = sum;


}
DLL_EXPORT void GetCommand(int *AI_OUT)
{
    AI_OUT[0] = WheelLeft;
    AI_OUT[1] = WheelRight;
    AI_OUT[2] = LED_1;
    AI_OUT[3] = MyState;

}




void Game0()
{
    if(step==0){
    LED_1=1;
    MyState=0;
    WheelLeft=1.5;
    WheelRight=1.5;
   if(delay(1000)==true){step=1;}
    }
    else if(step==1){
      LED_1=2;
      MyState=0;
      WheelLeft=0;
      WheelRight=0;
     if(delay(2000)==true){step=0;}
    }
    printf("Time: %d\n", Time);


}
//wheel speed in mm/s
double linear_v=100;

void Game1()
{
  static int count=0;
  static int stat=0;
  robot_heading=Compass-270;
  if(robot_heading<-180){robot_heading=robot_heading+360;}
  if(count<2){
  compute_reliable_pose();
}
  if(count==0&&go_to_goal(337, 98,pose_x,pose_y,robot_heading)==true){
    printf("waypoint 1 reached!\n");
    count=1;
  }
  // original 222,132
  if(count==1&&go_to_goal(224, 147,pose_x,pose_y,robot_heading)==true){
    printf("waypoint 2 reached\n");
    count=2;
  }


previous_x_pose=PositionX;
previous_y_pose=PositionY;
}
DLL_EXPORT void OnTimer(){
    switch (CurGame)
    {
        case 9:
            break;
        case 10:
            WheelLeft=0;
            WheelRight=0;
            LED_1=0;
            MyState=0;
            break;
        case 0:
            Game0();
            break;
        case 1:
            Game1();
            break;
        default:
            break;
    }
}

//returns true if delayed time has been reached.
bool delay(int milli_seconds){

  	static int call_count=0;
  	static clock_t start_time;
  	// Converting time into milli_seconds

  	// Storing start time
  	if (call_count == 0) {
  		start_time = clock();
  		call_count = 1;
  		return false;
  	}
  	//if we have reached the desired wait duration, do this.
  	else if (call_count==1&&((clock() < start_time + milli_seconds)!=true)) {
  		//we have reached the desired time.
  		call_count = 0;
  		return true;

  	}

  	else {
  		return false;
  	}
}

int mu_conversion(double wheel_velocity) {
  	//function returns the corresponding motor units.

  	//motor units and their corresponding angular velocities in (deg/s)
  	const static double one_mu = 190.9859317;
  	const static double two_mu = 381.9718634;
  	const static double three_mu = 572.9577951;
  	const static double four_mu = 763.9437268;
  	const static double five_mu = 954.9296586;
  	int final_mu = 0;
  	double original_wheel_velocity = 0;
  	//store original wheel velocity for negate.
  	original_wheel_velocity = wheel_velocity;
  	if (wheel_velocity < 0) { wheel_velocity = abs(wheel_velocity); }


  		 if (wheel_velocity == 0) { final_mu = 0; }
  	else if (wheel_velocity > 0 && wheel_velocity <= (one_mu/2.0)) { final_mu = 0; }
  	else if (wheel_velocity > (one_mu / 2.0) && wheel_velocity <= one_mu) { final_mu = 1; }
  	else if (wheel_velocity > one_mu&&wheel_velocity <= ((one_mu + two_mu) / 2.0)){ final_mu = 1; }
  	else if (wheel_velocity > ((one_mu + two_mu) / 2.0) &&wheel_velocity <= two_mu){ final_mu = 2; }
  	else if (wheel_velocity > two_mu &&wheel_velocity <= ((two_mu + three_mu) / 2.0)){ final_mu = 2; }
  	else if (wheel_velocity > ((two_mu + three_mu) / 2.0) && wheel_velocity <= three_mu) { final_mu = 3; }
  	else if (wheel_velocity > three_mu &&wheel_velocity <= ((three_mu + four_mu) / 2.0)) { final_mu = 3; }
  	else if (wheel_velocity > ((three_mu + four_mu) / 2.0) && wheel_velocity <= four_mu) { final_mu = 4; }
  	else if (wheel_velocity > four_mu && wheel_velocity <= ((four_mu + five_mu) / 2.0)) { final_mu = 4; }
  	else if (wheel_velocity > ((four_mu + five_mu) / 2.0) && wheel_velocity <= five_mu) { final_mu = 5; }
  	else if (wheel_velocity > five_mu) { final_mu = 5; }

  	//add back the neative sign if abs function was applied.
  	if (original_wheel_velocity < 0) { return final_mu*-1; }
  	else { return final_mu; }
}


void compute_cospace_velocities(double linear, double angular) {


	//compute left and right wheel velocities in (deg/s) based on linear V(mm/s)and angular W(deg/s)
	 double wheel_right_velocity = ((360.0*linear) / (60.0*pi))+(2*angular);
	 double wheel_left_velocity = ((360.0*linear) / (60.0*pi)) - (2 * angular);


	//convert wheel angular velocities to their closest corresponding motor units.
	WheelLeft = mu_conversion(wheel_left_velocity);
	WheelRight = mu_conversion(wheel_right_velocity);

}


bool go_to_goal(double goal_x, double goal_y,double robot_x,double robot_y,double robot_heading) {

  	double goal_heading = 0;
  	double heading_error = 0;
  	double y_diff = goal_y - robot_y;
  	double x_diff = goal_x - robot_x;
  	double distance_to_goal = 0;
  	bool goal_reached = false;

  	// the threshold where we should turn on the spot first before we head to the goal. (deg)
  	static double turn_threshold = 100;
  	//goal tolerance in cm. if its less than this value, then we can take it that the robot has reached the goal
  	static double goal_tolerance = 5;
  	//proportional gain
  	static const double Kp = 3.0;
  	//distance to goal where the robot would start to slow down
  	static double goal_speed_threshold = 20;


  	//angular control signal (deg/s)
  	double w_s = 0;
  	//linear control signal (mm/s)
  	double v_s = 0;



  	//compute angular goal in (deg)
  	goal_heading = atan2(y_diff, x_diff)*(180.0 / pi);

  	//compute heading error
  	heading_error = goal_heading - robot_heading;

  	// limit heading error between +-180
  	if (heading_error < -180.0) { heading_error = heading_error + 360.0; }
  	if (heading_error > 180.0) { heading_error = heading_error - 360.0; }

  	//feed error into proportional controller to calculate angular control signal
  	w_s = Kp*heading_error;


  	//compute absolute distance between robot and goal in cm
  	distance_to_goal = sqrt(pow(x_diff, 2) + pow(y_diff, 2));

  	//if the heading error is large, we would want to turn on the spot first.
  	if (abs(heading_error) > turn_threshold) { v_s = 0; goal_reached = false; }
  	// if the bot is far from the goal, move at a faster speed
  	else if (distance_to_goal >= goal_speed_threshold) { v_s = 300; goal_reached = false;}
  	// if the bot is approaching the goal, slow down.
  	else if (distance_to_goal >=goal_tolerance&&distance_to_goal < goal_speed_threshold) { v_s = 100; goal_reached = false;}
  	//if the bot has reached the goal, stop.
  	else if (distance_to_goal < goal_tolerance) { v_s = 0; w_s = 0; goal_reached = true;}
      //printf("angular control signal: %f\n",w_s);
    //  printf("linear control signal: %f\n", v_s);
  //  printf("distance to goal: %f\n", distance_to_goal);
    //printf("robot_x: %f\n", robot_x);
      //printf("robot_y: %f\n", robot_y);
  	//send speed commands here
    compute_cospace_velocities(v_s, w_s);
  	return goal_reached;
}

void pseudo_odometry(double current_heading){
    //initialise the start position
    static double current_x=0;// x and y estimate in mm
    static double current_y=0;
    double left_wheel_disp=0;
    double right_wheel_disp=0;
    double centre_disp=0;

    //init position first.
    if(init_odometry==true){
    current_x=previous_x_pose*10.0;
    current_y=previous_y_pose*10.0;
    printf("initial pose: X: %f Y: %f\n", current_x,current_y);
    init_odometry=false;
    }


    //get wheel displacements in mm after 61.22ms
     left_wheel_disp=wheel_displacements(WheelLeft);
     right_wheel_disp=wheel_displacements(WheelRight);

    //compute displacement of centre of robot
    centre_disp=(left_wheel_disp+right_wheel_disp)/2.0;

    //convert current heading from deg to radians
    current_heading=current_heading*(pi/180.0);

    //calculate new coords
    current_x+=centre_disp*cos(current_heading);
    current_y+=centre_disp*sin(current_heading);

    estimated_x=current_x/10.0;
    estimated_y=current_y/10.0;
  //  printf("current odom x and y estimate is: %f  %f\n",current_x/10.0,current_y/10.0);

}

// function returns the linear distance travelled by each wheel given its motor units, after 61.22 ms.
double wheel_displacements(int motor_units) {
	return motor_units*100.0*0.06122;
}


void compute_reliable_pose(){
  static int state=0;

  //if there are missing coordinates, then we will switch over to the odom estimate.
  if((PositionX==0||PositionY==0)&&state==0){
  //init odom using previous known pose
  init_odometry=true;
  //compute current pose with odometry
  pseudo_odometry(robot_heading);
  //update reliable pose variables.
  pose_x=estimated_x;
  pose_y=estimated_y;
  state=1;
  }
  //keep providing odom estimate poses while we are still in no signal zone.
  else if(state==1){
    //compute current pose with odometry
    pseudo_odometry(robot_heading);
    //update reliable pose variables.
    pose_x=estimated_x;
    pose_y=estimated_y;
    printf("no server pose. using odom pose estimate X: %f Y: %f\n", pose_x,pose_y);
    //when signal is back, exit odom pose estimate.
    if(PositionX!=0&&PositionY!=0){state=0;}

  }

  // if theres no missing coordinates, just take the existing pose values provided by the server.
  if(PositionX!=0&&PositionY!=0){
  printf("using server pose X: %f Y: %f\n", pose_x,pose_y);
  pose_x=PositionX;
  pose_y=PositionY;
  }

}
