/*
Author: Faizal Patel
Lisbon 2011
*/
double PID_attitude(double PID_error,double dt, int PID_Case){
   double output = 0;
   double derivative;
   derivative = 1000.000 * (PID_error - LastError[PID_Case]) / dt;
   LastError[PID_Case] = PID_error;
   output = kp[PID_Case] * PID_error;
   Integrator[PID_Case] = Integrator[PID_Case] + PID_error * dt / 1000.000;
  
   Integrator[PID_Case] = constrain(Integrator[PID_Case], -1, 1);
  
   output += Integrator[PID_Case] * ki[PID_Case];
   output += derivative * kd[PID_Case];
   
  
     output=constrain(output,-1,1);
 
   return output;
}


double PID_nav(double PID_error,double dt, int PID_Case){
   double output = 0;
   double derivative;
   derivative = 1000.000 * (PID_error - LastError[PID_Case]) / dt;
   LastError[PID_Case] = PID_error;
   output = kp[PID_Case] * PID_error;
   Integrator[PID_Case] = Integrator[PID_Case] + PID_error * dt / 1000.000;
   if(PID_Case==3){
    Integrator[PID_Case] = constrain(Integrator[PID_Case], -35, 35);
   }
   if(PID_Case==2){
    Integrator[PID_Case] = constrain(Integrator[PID_Case], -10, 10);
   }
  
   output += Integrator[PID_Case] * ki[PID_Case];
   output += derivative * kd[PID_Case];
   if(PID_Case==3){
   output=constrain(output,-35,35);
   }
   if(PID_Case==2){
   output=constrain(output,-10,10);
   }
  
    
 
   return output;
}


void resetIntehrator(int a){
 
      Integrator[a]=0; 
      LastError[a]=0;
  
}


double getBearing(double lat1, double lat2, double lon1, double lon2){
  double dlon = lon2 - lon1;
  double offx = cos(ToRad(lat1)) * sin(ToRad(lat2)) - 
  sin(ToRad(lat1)) * cos(ToRad(lat2)) * cos(ToRad(dlon)); 
  double offy = sin(ToRad(dlon))*cos(ToRad(lat2));
  double bear=atan2(offy,offx)*180/PI;
  if(bear<0)
  {
   bear+=360; 
  }
  
  return bear;
       }
       
       
       
double AltDistance(double alt1, double alt2){
    return abs(alt2 - alt1);
}       

double distance(double lat1, double lat2, double lon1, double lon2, int index){
  double x=69.1*(lon2-lon1)*cos(ToRad(lat2-lat1));
  double y=69.1*(lat2-lat1);
  return 1609.344*sqrt(sq(x)+sq(y));
  
  
 
}

void autopilot(double delta_t){
 
 kp[2]=1.3;
 ki[2]=0;
 kd[2]=0;

  float Thr;
  float navRoll;
  float navPitch;
  float Elevator;
navRoll=map(input_rc[0],1094,1927,-30000,30000); 
navPitch=map(input_rc[1],1094,1927,-20000,20000);
 
//RefRoll=navRoll/1000;//RefRoll;// -35;///1000;//;
//RefPitch=navPitch/1000;//0;

//Naviagtion
//correct_heading_reference();
//RefRoll=PID_nav(heading_error,delta_t,3);


joinedwing.aileron=RefRoll;//PID_attitude((RefRoll-imu.r),delta_t,0);
joinedwing.elevator=RefPitch;//PID_attitude((RefPitch-imu.p),delta_t,8);
joinedwing.rudder=0;//PID_attitude(RefYaw-imu.y,delta_t,4);
Thr=map(input_rc[2],1514,1094,0,1000);
Thr=constrain(Thr,0,1000);
joinedwing.throttle=1;//Thr/1000;
HomeDistance=distance(gps.lat, waypoint.lat[0], gps.lon, waypoint.lon[0],WayPointIndexer); 
NextWaypointDist=distance(gps.lat, waypoint.lat[WayPointIndexer], gps.lon, waypoint.lon[WayPointIndexer],WayPointIndexer);

 switch(command){
   case 4:
   joinedwing.throttle=subcommand;
   break;
   
   default:
   
   break;
  }

}


void UAV_Tasks(double delta_t)
{
  switch(command){

case 0:
flight_status=3;  //Loiter
refLoiter=subcommand;
return;
break;

case 1:  //gohome
flight_status=4;  //return home
return;
break;

case 2: //hdg
     flight_status=5;
     RefHead=subcommand;
return;    
break;


case 3:  //alt
waypoint.alt[WayPointIndexer]=subcommand;
return;
break;


case 5:
flight_status=1;  //navi
return;
break;

case 6:  //landnow
    flight_status=2;
 return;
    break;
    
    
    
default:

return;
break;





}  
  
}

void uav(double delta_t){
  
if(takeoff_x==1)
{
  UAV_Tasks(delta_t);
}
  
switch(flight_status){
       case 0:
             takeoff(delta_t);
             RefYaw=0;
           if (gps.alt<=5)
              joinedwing.rudder=PID_attitude(RefYaw-imu.y,delta_t,4);
             else{
               joinedwing.rudder=0;//PID_attitude(0-imu.ay,delta_t,6);
             }
           // joinedwing.rudder=0;
       break;
      
       case 1:
          navigate(delta_t);
          joinedwing.rudder=0;//PID_attitude(0-imu.ay,delta_t,6);
          RefYaw=0;
          //   if(HomeDistance>=1500)
            //          {flight_status=3; }
                
            //if (abs(RefRoll)>=30){}
            //else{joinedwing.rudder=0;}
           //PID_attitude(0-imu.ay,delta_t,6);
            joinedwing.rudder=0;
        
       break;
       case 2:
           land(delta_t);
           RefYaw=0;
            //PID_attitude(RefYaw-imu.y,delta_t,4);
            
       break;
       case 3:
            loiter(delta_t);
           // joinedwing.rudder=PID_attitude(0-imu.ay,delta_t,6);
            flight_status=3;  
            joinedwing.rudder=0;
           break; 
     case 4:
     return_home();
     break;
     
     case 5:
     on_course();      
     joinedwing.rudder=0;
     break;
     
     
     
     default:
       flight_status=0;
     break;  
      
      
}
   

 switch(command)
{
case 4:
joinedwing.throttle=subcommand;
joinedwing.throttle=constrain(joinedwing.throttle,0,1);
break;

case 8:
joinedwing.throttle=1;
break;

case 7:
joinedwing.throttle=0;
break;

}  
  
HomeDistance=distance(gps.lat, waypoint.lat[0], gps.lon, waypoint.lon[0],WayPointIndexer); 
NextWaypointDist=distance(gps.lat, waypoint.lat[WayPointIndexer], gps.lon, waypoint.lon[WayPointIndexer],WayPointIndexer); 
  stabilize(delta_t); 

}


void manual_flight(double delta_t){
  float Ailr;
  float Elevator;
  float Thr;
  float Rud;
  
 
Ailr=map(input_rc[0],1094,1927,-1000,1000);
Rud=map(input_rc[2],1094,1927,-1000,1000);
Elevator=map(input_rc[1],1927,1094,-1000,1000);

Thr=map(input_rc[2],1514,1094,0,1000);
Thr=constrain(Thr,0,1000);
joinedwing.aileron=Ailr/1000;
joinedwing.elevator=Elevator/1000;
joinedwing.throttle=Thr/1000;
joinedwing.rudder=0;//Rud/1000;
HomeDistance=distance(gps.lat, waypoint.lat[0], gps.lon, waypoint.lon[0],WayPointIndexer); 
NextWaypointDist=distance(gps.lat, waypoint.lat[WayPointIndexer], gps.lon, waypoint.lon[WayPointIndexer],WayPointIndexer);
}
