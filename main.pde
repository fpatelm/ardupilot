/*
Author: Faizal Patel
Lisbon 2011
*/
#include <math.h>
#include "defines.h"
#include "TinyGPS.h"
#include <Servo.h> 
#include <avr/pgmspace.h>
unsigned long timer=0;          //timer
unsigned long  delta_t;         //delta time or how long it takes to execute data acquisition 

double times=millis();

IMU_Data imu;
GPS_Data gps;
Aircraft joinedwing;
WayPointsMap waypoint;
Servo C1_R;
Servo C2_R;
Servo C3_R;
Servo C4_R;
Servo C5_R;

Servo C1_L;
Servo C2_L;
Servo C3_L;
Servo C4_L;
Servo C5_L;
Servo C6; //Engine Controller

#define IO_Driver Serial
#define XBee_Driver Serial2


void setup()
{
  analogReference(EXTERNAL);     //using external analog ref of 3.3V for ADC scaling
  IO_Driver.begin(115200);         //
  XBee_Driver.begin(115200);
  DDRC = B00000000;              //make all analog ports as inputs - just in case.... 
  init_analog_ports();
  init_rc_input();
  delay(20);
 // Serial.print("10 sec to calibrate sensores - do not move\n");
  for(int i=0;i<150;i++)
       { init_g_a(); } 
  init_var();
  init_servos();
delay(5);
  pinMode(13,OUTPUT);
  timer=millis(); 
 // Serial1.println("Xbee on");
 //read_serial();

}





void loop()
{ //Serial1.println("Xbee on");
  
  //Static Devices
//

  delta_t = millis() - timer;
  timer=millis(); 

//input_rc[4]=1314; // Auto-pilot  

//  input_rc[5]=1000; //Real
//read_rc_receiver();
 // input_rc[4]=pulseIn(ch5,HIGH);
 // input_rc[5]=pulseIn(ch6,HIGH);
  input_rc[4]=1110; //UAV 
  input_rc[5]=1634; //Sim
 // FlightGear Aircraft
//if (input_rc[5]>=1614){
  if (IO_Driver.available()>=0){
    read_serial();
 delay(20);
      }
     
    
    if(input_rc[4]>=1614)
    {
    input_rc[0]=pulseIn(ch1,HIGH);
   input_rc[1]=pulseIn(ch2,HIGH);
    input_rc[2]=pulseIn(ch3,HIGH);
    input_rc[3]=pulseIn(ch4,HIGH);
     manual_flight(delta_t);

double ang=30*joinedwing.aileron;
              double angs=30*joinedwing.elevator;


  C1_R.write(ang+90);
    C2_R.write(angs+90);
   send_data();
    }
    
    if(input_rc[4]<=1614 && input_rc[4]>=1214)
    {
      imu_func(delta_t); 
      input_rc[0]=pulseIn(ch1,HIGH);
      input_rc[1]=pulseIn(ch2,HIGH);
      input_rc[2]=pulseIn(ch3,HIGH);
      input_rc[3]=pulseIn(ch4,HIGH);
      autopilot(delta_t);
       
       //double ang=30*joinedwing.aileron;
double ang=30*joinedwing.aileron;
              double angs=30*joinedwing.elevator;


  C1_R.write(ang+90);
    C2_R.write(angs+90);
  send_data();
   
    }
    
      if (input_rc[4]<=1114)
      {
      uav(delta_t);  
       double ang=30*joinedwing.aileron;
              double angs=30*joinedwing.elevator;


  C1_R.write(ang+90);
    C2_R.write(angs+90);
      send_data();
  //calc_yaw(delta_t);
      }
   

   

   
/*
//Real Aircraft
if (input_rc[5]<=1114){
    seed_gps();
    imu_func(delta_t); 
    if(input_rc[4]>=1614)
      {
         manual_flight(delta_t);
      }
    
  if(input_rc[4]<=1614 && input_rc[4]>=1214)
      {
          autopilot(delta_t);
           double ang_a=30*joinedwing.aileron;
           double ang_e=30*joinedwing.elevator;
           C1_R.write(ang_a+90);
           C2_R.write(ang_e+90);
      }
    
   if (input_rc[4]<=1114)
        {
          uav(delta_t);  
           double ang_a=30*joinedwing.aileron;
           double ang_e=30*joinedwing.elevator;
           C1_R.write(ang_a+90);
           C2_R.write(ang_e+90);
        }
      
     
  
  
  //  move_servos();
    send_data();
  
}
//  double ang=30*joinedwing.aileron;//map(joinedwing.elevator,-1,1,-1000,1000);
//   ang=map(ang,-1000,1000,-30,30);
  // C1_s1.write(ang+90);
   //Serial2.println(ang+90);
 
*/

}
