/*
/defines.h
Author: Faizal Patel
Lisbon 2011
Mode:
    0: Simulated with Flight Gear
ModelType:
    1: Conventional Aircraft - One Aileron, One Elevator, One Rudder

*/

#define ToRad(x) (x*PI)/180
#define ToDeg(x) (x*180)/PI
#define C1_p1 2
#define C2_p1 3
#define C3_p1 4
#define C4_p1 5
#define C5_p1 6

#define C1_p2 7
#define C2_p2 8
#define C3_p2 9
#define C4_p2 10
#define C5_p2 11
#define C6_p 12

#define ch1 22
#define ch2 23
#define ch3 26
#define ch4 27
#define ch5 30
#define ch6 31
#define ch7 34


#define Mode 0
#define ModelType 0
unsigned long input_rc[7]={0,0,0,0,0,0,0};
//Sensor variables
int adGy,adGx,adGz,adAz,adAy,adAx;
int adGy0,adGx0,adGz0,adAz0,adAy0,adAx0;
double rateGy,rateGx,rateGz,angleAz,angleAy,angleAx;
double angleGy,angleGx,angleGz,valAz,valAy,valAx;
//*************
double roll,pitch,yaw;
//Ail, Elev, Alt, Head
#define number_controllers 9
double LastError[number_controllers]={0,0,0,0,0,0,0,0,0};
double Integrator[number_controllers]={0,0,0,0,0,0,0,0,0}; 
double yaw_angg=0;
double kp[number_controllers]={
            0.056,  //Roll-Aileron nav           0
            -0.06, //Pitch-Elevator        1  -0.0378
            1.5,   //Alt-Pitch              2 
            0.7,   //Head-Roll               3
            0.054, //Yaw-Rudder              4
            0.00053, //Alt-Throttle          5   0.00053
            -5.590,   // AcelerY                6
            0.010,  //Autopilot roll controller
            -0.038};   //Autopilot pitch controller
double ki[number_controllers]={
            0.02396 , //Roll-Aileron nav   0.02396        0
              0, //Pitch-Elevator        1  -0.059731
             0.066956, //Alt-Pitch              2 
             0.1571, //Head-Roll               3 //0.1571
              0.05990 ,//        Yaw-Rudder              4     v
             0.2142,  //Alt-Throttle           5   0.2142
             0,      //AcelerY                6    -0.6
             0.01396,//Autopilot roll controller
             -0.019721};//Autopilot pitch controller
double kd[number_controllers]={
            0, //0.0000054
            0, //-0.059731
            0,
            0,
            0,
            0,
            0.0000054,
            -0.01086};

double vect[13];
char buf[500];
char *str;
double RefRoll;
double RefPitch;
double RefAlt;
double RefHead;
double HomeDistance;
double NextWaypointDist;
double heading_error;
const double t7				= 10000000.0;
int 	wp_radius			= 50;			// meters - set by config tool!
//Flight Variables
int flight_status=0;
double TakeOffAltitude=100; // (ft)
double th0=0.5;
double delta_th;
int inside_land_area=0;
int command=9;
float subcommand=0;
float refLoiter=0;
int takeoff_x=0;

//

//

//********************

struct IMU_Data{
  double r;
  double y;
  double p;
  double ay;
};

struct WayPointsMap{
  int length;
  double lat[60];
  double lon[60];
  double alt[60];
  double latHome;
  double lonHome;
  double altHome;
};
double   yaw_angle;
double RefYaw;
int WayPointIndexer=1;
struct GPS_Data{
  
 double lat;
 double lon; 
 double alt;
 double gs;
 double gc;
  
};
struct Aircraft{
  double aileron;
  double elevator;
  double rudder;
  double throttle;
};
