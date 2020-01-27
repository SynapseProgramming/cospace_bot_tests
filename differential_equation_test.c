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
int count=0;
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
double WheelLeft = 0;
double WheelRight = 0;
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
 int start_x=0;
 int start_y=0;
 int start_theta=0;
 int state=0;
 int coord_x_diff=0;
 int coord_y_diff=0;
 int theta_diff=0;

//wheel speed in mm/s
double linear_v=100;

void Game1()
{
    if(state==0){
      compute_cospace_velocities(linear_v, 0);
      LED_1=0;
      if(delay(500)==true){state=1;}
      printf("moving at constant speed for 500ms\n");
    }
    else if (state==1){
    compute_cospace_velocities(linear_v, 0);
    LED_1=0;
    start_x= PositionX;//over here, we would want to store the initial coordinates
    start_y= PositionY;
    printf("stored starting position!\n");
    state=2;
}
    else if(state==2){
      compute_cospace_velocities(linear_v, 0);
      LED_1=0;
      printf("moving at constant speed for 1000ms\n");
      if(delay(1000)==true){state=3;}

    }
    else if(state==3){
      compute_cospace_velocities(0, 0);
      LED_1=0;
      coord_x_diff=PositionX-start_x;
      coord_y_diff=PositionY-start_y;
      printf("x difference: %d\n",coord_x_diff);
      printf("y difference: %d\n",coord_y_diff);
      state=4;
    }
    else if (state==4){
      compute_cospace_velocities(0, 200);
      LED_1=0;
      if(delay(500)==true){state=5;}
      printf("turning at constant speed for 500ms\n");
    }
    else if (state==5){
      compute_cospace_velocities(0, 200);
      LED_1=0;
      start_theta=Compass;
      printf("stored starting position!\n");
      state=6;
    }
    else if (state==6){
      compute_cospace_velocities(0, 200);
      LED_1=0;
      printf("moving at constant speed for 1000ms\n");
      if(delay(1000)==true){state=7;}
    }
    else if (state==7){
      compute_cospace_velocities(0, 0);
      LED_1=0;
      theta_diff=start_theta-Compass;
      printf("theta difference: %d\n",theta_diff);
      state=8;
    }
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
          //test statements to determine call frequency.
        //    printf("loop count: %d\n", count);
          //  count++;
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
bool delay(int milli_seconds)

{

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
