/*
Author: Faizal Patel
Lisbon 2011
*/
void takeoff(double delta_t){
HomeDistance=distance(gps.lat, waypoint.lat[0], gps.lon, waypoint.lon[0],WayPointIndexer); 
joinedwing.throttle=1;
RefRoll=0;
RefAlt=TakeOffAltitude;
RefPitch=0;//PID_nav(RefAlt-gps.alt,delta_t,2);
RefPitch=constrain(RefPitch,0,15);
if(gps.alt>RefAlt)
{
      flight_status=1;
      takeoff_x=1;
      //resetIntehrator(0);   
      //resetIntehrator(1);   
}
else{flight_status=0;}
NextWaypointDist=distance(gps.lat, waypoint.lat[WayPointIndexer], gps.lon, waypoint.lon[WayPointIndexer],WayPointIndexer); 

}


void stabilize(double delta_t){
    
joinedwing.aileron=PID_attitude((RefRoll-imu.r),delta_t,0);
joinedwing.elevator=PID_attitude((RefPitch-imu.p),delta_t,1);
}



void correct_heading_reference(){
   float error=RefHead-gps.gc;  
   heading_error=error;
   if(RefHead>270 && gps.gc<=90)
   {
      heading_error=-error;
   }
   
   if(RefHead<=270 && gps.gc>90)
   {
     heading_error=error; 
   }
   
      if(gps.gc>270 && RefHead<=90)
   {
      heading_error=-error;
   }
   
   if(gps.gc<=270 && gps.gc>90 && RefHead>90 && RefHead<=270)
   {
     heading_error=error; 
   }
   
   if(RefHead>270 && gps.gc>90)
   {
      heading_error=error;
   }
   if(RefHead<=270 && gps.gc<=90)
   {
      heading_error=error;
   }  
   
   //resetIntehrator(3); 
   //resetIntehrator(0); 
   /*
    if( (gps.gc<90 && 270<RefHead<360) || (270<gps.gc<360 && RefHead<90) || (gps.gc<45 && 225<RefHead) || (gps.gc<135 && 315<RefHead) || (gps.gc<225 && 45<RefHead) || (315<gps.gc && RefHead<135) ||   (gps.gc<45 && RefHead<225) || (gps.gc>45 && RefHead>225)||(gps.gc>135 && RefHead>315)||(gps.gc>225 && RefHead>45)||(gps.gc>315 && RefHead>135)){
    heading_error=-error;
   
  }
   
   
   if((gps.gc<90 && RefHead<90)||(gps.gc>270 && gps.gc<90 && 90<RefHead<180) || (90<gps.gc<180 && RefHead<90) || (90<gps.gc<180 && 90<RefHead<180) ||(90<gps.gc<180 && 180<RefHead<270) ||
   (180<gps.gc<270 && 90<RefHead<180) || (180<gps.gc<270 && 180<RefHead<270) || (180<gps.gc<270 && 270<RefHead<360) || (270<gps.gc<360 && 180<RefHead<270) || (270<gps.gc<360 && 270<RefHead<360) ||
   (45<gps.gc && RefHead<225) || (135<gps.gc && RefHead<315) || (225<gps.gc && RefHead<45) || (gps.gc<315 && 135<RefHead)||(gps.gc<135 && RefHead<315) || (gps.gc<225 && RefHead<45) || (gps.gc<315 && RefHead<135)){
   heading_error=error;     
 
   }
   */
}
/*
void altitude_follower(){
  RefPitch=PID_nav(int(RefAlt-gps.alt),delta_t,2);
  RefPitch=constrain(RefPitch,-10,10);
  
  if(gps.alt<=RefAlt && abs(RefAlt-gps.alt)>=50){
    joinedwing.throttle=1;
  }
  
    if(gps.alt<=RefAlt && abs(RefAlt-gps.alt)<50){
    joinedwing.throttle=0.9;
  }
  
      if(gps.alt>=RefAlt && abs(RefAlt-gps.alt)>=50){
    joinedwing.throttle=0.55;
  }
      if(gps.alt>=RefAlt && abs(RefAlt-gps.alt)<50){
    joinedwing.throttle=0.65;
  }  
      if(abs(RefAlt-gps.alt)<5){
    joinedwing.throttle=0.70;
    RefPitch=0;
  }
}

*/
void navigate(double delta_t){
  
  RefHead=getBearing(gps.lat, waypoint.lat[WayPointIndexer], gps.lon, waypoint.lon[WayPointIndexer]);    
  RefAlt=waypoint.alt[WayPointIndexer];
  correct_heading_reference();
  /*
  if((gps.gc>270 && RefHead<360 && RefHead>270)||(gps.gc>90 && gps.gc<270 && RefHead>90 && RefHead<270)||(gps.gc<90 && gps.gc>0 && RefHead>0 && RefHead<90)||(gps.gc<270 && gps.gc>90 && RefHead>90 && RefHead<270)|| (gps.gc>90 && RefHead<360 && RefHead>270)||(RefHead<90 && gps.gc<180)||(gps.gc>0 && RefHead<280) ){
   heading_error=RefHead-gps.gc; 
  }
  
  if((gps.gc>270 && RefHead<90 && RefHead>0)||(gps.gc<90 && RefHead<0 && RefHead>270)||(RefHead<90 && gps.gc>180)){
   heading_error=gps.gc-RefHead;   
}

*/
  
  
  // RefRoll=constrain(RefRoll,-30,30);
  //altitude_follower();
  /*
  if(abs(RefAlt-gps.alt<100))
  {
    if(RefAlt>gps.alt)
    {
      joinedwing.throttle=0.7; 
      RefPitch=constrain(RefPitch,-8,8);
    }
    if(RefAlt<gps.alt)
    {
       joinedwing.throttle=0.55;
       RefPitch=constrain(RefPitch,-5,5);
    }
  }
 
  
  if(abs(RefAlt-gps.alt>100))
  {
    if(RefAlt>gps.alt)
    {
      joinedwing.throttle=0.99; 
      RefPitch=constrain(RefPitch,-15,15);
    }
    if(RefAlt<gps.alt)
    {
       joinedwing.throttle=0.4;
       RefPitch=constrain(RefPitch,-15,15);
    }    
     
  }
  
  */
  NextWaypointDist=distance(gps.lat, waypoint.lat[WayPointIndexer], gps.lon, waypoint.lon[WayPointIndexer],WayPointIndexer); 
  if(NextWaypointDist<wp_radius)
  {
          waypoint_reached();
  }
      RefRoll=PID_nav(heading_error,delta_t,3);
      delta_th=PID_attitude(RefAlt-gps.alt,delta_t,5);
      joinedwing.throttle=th0+delta_th;
      joinedwing.throttle=constrain(joinedwing.throttle,0,1);
      RefPitch=2;PID_nav(int(RefAlt-gps.alt),delta_t,2);
     // RefPitch=constrain(RefPitch,-2,2);
}    
  
void waypoint_reached(){
  WayPointIndexer++; 
  //resetIntehrator(0);
  //resetIntehrator(1);
  //resetIntehrator(2);
   // resetIntehrator(3);
     // resetIntehrator(4);
       // resetIntehrator(5);
  //Check last waypoint
  
//  float dist=distance(gps.lat, waypoint.lat[0], gps.lon, waypoint.lon[0],WayPointIndexer); 
  if(WayPointIndexer==(waypoint.length))// && dist<=2000) //last 3 coordinates is the landing path
  {
            flight_status=2;
  }
  else{flight_status=1;}
}


void loiter(double delta_t){
 
  /*
   if(HomeDistance>300 && gps.alt<100)
  {
    RefRoll=30;
    RefAlt=500;
    altitude_follower();
    flight_status=3; //Loiter to correct altitude
  }
  
  if(HomeDistance<300 && gps.alt>500)
  {
    RefRoll=30;
    RefPitch=PID_nav(100-gps.alt,delta_t,2);
    flight_status=3; //Loiter to correct altitude
  }
  else
  {
     flight_status=2;
  }
  
  NextWaypointDist=distance(gps.lat, waypoint.lat[WayPointIndexer], gps.lon, waypoint.lon[WayPointIndexer],WayPointIndexer); 
  HomeDistance=distance(gps.lat, waypoint.lat[0], gps.lon, waypoint.lon[0],WayPointIndexer); 
  delta_th=PID_attitude(RefAlt-gps.alt,delta_t,5);
      joinedwing.throttle=th0+delta_th;
    */  
  RefAlt=waypoint.alt[WayPointIndexer];
  NextWaypointDist=distance(gps.lat, waypoint.lat[WayPointIndexer], gps.lon, waypoint.lon[WayPointIndexer],WayPointIndexer); 
  HomeDistance=distance(gps.lat, waypoint.lat[0], gps.lon, waypoint.lon[0],WayPointIndexer); 
  delta_th=PID_attitude(RefAlt-gps.alt,delta_t,5);
  joinedwing.throttle=th0+delta_th+0.5;
  RefRoll=refLoiter;
}


void return_home(){
  
RefAlt=waypoint.alt[waypoint.length] ;
delta_th=PID_attitude(RefAlt-gps.alt,delta_t,5);
joinedwing.throttle=th0+delta_th;
RefHead=getBearing(gps.lat, waypoint.lat[waypoint.length], gps.lon, waypoint.lon[waypoint.length]);    
correct_heading_reference();
RefRoll=PID_nav(heading_error,delta_t,3);
float descend_angle=atan2(gps.alt,HomeDistance);
RefPitch=-descend_angle;


}


void land(double delta_t){
  NextWaypointDist=distance(gps.lat, waypoint.lat[WayPointIndexer], gps.lon, waypoint.lon[WayPointIndexer],WayPointIndexer); 
  HomeDistance=distance(gps.lat, waypoint.lat[0], gps.lon, waypoint.lon[0],WayPointIndexer); 
  //float land_dist=gps.alt/atan(imu.p);
  //float descend_angle=atan2(gps.alt,HomeDistance);
  //RefHead=getBearing(gps.lat, waypoint.lat[WayPointIndexer], gps.lon, waypoint.lon[WayPointIndexer]);    
  //correct_heading_reference();
  //RefRoll=PID_nav(heading_error,delta_t,3);
  //RefPitch=-descend_angle;
  // joinedwing.throttle=0.5;
  //RefAlt=tan(5*PI/180)*HomeDistance;
  //RefAlt=constrain(RefAlt,0,waypoint.alt[WayPointIndexer+1]);
  
  delta_th=PID_attitude(RefAlt-gps.alt,delta_t,5);
  joinedwing.throttle=th0+delta_th;
  joinedwing.throttle=constrain(joinedwing.throttle,0,1);
  RefHead=getBearing(gps.lat, waypoint.lat[WayPointIndexer+1], gps.lon, waypoint.lon[WayPointIndexer+1]);    
  correct_heading_reference();
  RefRoll=PID_nav(heading_error,delta_t,3);
  RefPitch=3;
  
  if(gps.alt<=5)
  {
   joinedwing.rudder=PID_attitude(RefYaw-imu.y,delta_t,4); 
  }
  
  else{
    joinedwing.rudder=0;
  }

 flight_status=5;
   
  
  
  /*
  
  if(HomeDistance>=1000)
  {
   RefAlt=1000;
  }
  
  if(HomeDistance<1000)
  {
   // RefAlt=abs(tan(9)*HomeDistance);
   descend_angle=atan2(gps.alt,HomeDistance);
   RefPitch=-descend_angle;
   RefHead=getBearing(gps.lat, waypoint.lat[WayPointIndexer], gps.lon, waypoint.lon[WayPointIndexer]);    
   correct_heading_reference();
   
   if(gps.alt>200)
     RefRoll=PID_nav(heading_error,delta_t,3);
   else
   RefRoll=0;
   RefAlt=0;
  }
/*
  if(HomeDistance>300 && gps.alt<100)
  {
    flight_status=3; //Loiter to correct altitude
  }
  
  if(HomeDistance<300 && gps.alt>500)
  {
    flight_status=3; //Loiter to correct altitude
  }
  if(gps.alt>100 && gps.alt<300)
  {
    RefRoll=PID_nav(heading_error,delta_t,3);
    RefPitch=4;
    joinedwing.throttle=0.2;
  }
   if(gps.alt>35 && gps.alt<=100)
  {
    RefRoll=0; 
    RefPitch=4;
    joinedwing.throttle=0.1;
  }
  if(gps.alt<=35)
  {
    RefRoll=0;
    RefPitch=3;
    joinedwing.throttle=0;
    joinedwing.rudder=PID_attitude(0-imu.y,delta_t,4);
  }
   if(gps.alt<=10)
  {
    RefRoll=0;
    RefPitch=0;
    joinedwing.throttle=0;
    joinedwing.rudder=PID_attitude(0-imu.y,delta_t,4);
  }
  

  
  /*
  if(gps.alt>=2010)
  {
    RefAlt=2000;
    RefHead=getBearing(gps.lat, waypoint.lat[WayPointIndexer], gps.lon, waypoint.lon[WayPointIndexer]);    
    correct_heading_reference();
    RefRoll=PID_nav(heading_error,delta_t,3);
    //joinedwing.throttle=0.8; 
  }
  
  
  if(gps.alt<=2000){    
    RefAlt=300;
    RefPitch=PID_nav(RefAlt-gps.alt,delta_t,2);
    RefPitch=constrain(RefPitch,-15,0);
    RefHead=getBearing(gps.lat, waypoint.lat[WayPointIndexer], gps.lon, waypoint.lon[WayPointIndexer]);    
    correct_heading_reference();
    RefRoll=PID_nav(heading_error,delta_t,3);
    //joinedwing.throttle=0.7;
  } 
 if(NextWaypointDist<=500 || inside_land_area==1){
      inside_land_area=1;
     //landinf process
      RefAlt=0;
      RefRoll=0; 
      RefPitch=-15;
      joinedwing.throttle=0.4;
    if(gps.alt<100 && gps.alt>20) {
      RefPitch=-5;
      RefRoll=0;
      joinedwing.throttle=0;
    }
  if(gps.alt<20)
    {
      RefPitch=2;
      RefRoll=0;
      joinedwing.throttle=0;
    }
    
  }
    
  
 
  if(inside_land_area==0)
  {
    RefPitch=PID_nav(RefAlt-gps.alt,delta_t,2);
    RefPitch=constrain(RefPitch,-10,10); 
    RefHead=getBearing(gps.lat, waypoint.lat[WayPointIndexer], gps.lon, waypoint.lon[WayPointIndexer]);    
    correct_heading_reference();
    RefRoll=PID_nav(heading_error,delta_t,3);
  }
*/  
//  altitude_follower();
/*
  flight_status=2;
  delta_th=PID_attitude(RefAlt-gps.alt,delta_t,5);
  joinedwing.throttle=th0+delta_th;
  */
}



void on_course(){
      
      correct_heading_reference();
      RefRoll=PID_nav(heading_error,delta_t,3);
      RefAlt=waypoint.alt[WayPointIndexer];
      delta_th=PID_attitude(RefAlt-gps.alt,delta_t,5);
      joinedwing.throttle=th0+delta_th;
      joinedwing.throttle=constrain(joinedwing.throttle,0,1);
      RefPitch=PID_nav(int(RefAlt-gps.alt),delta_t,2);
      RefPitch=constrain(RefPitch,-2,2); 
      flight_status=5;
}

