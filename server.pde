/*
Author: Faizal Patel
Lisbon 2011
*/


void read_serial(){
  GetString(buf,sizeof(buf));
  delay(30);
  decode_serial(buf);
//  delay(5);
}
void GetString(char *buf, int bufsize){
 int i;
 for(i=0;i<bufsize-1;i++){
 while(IO_Driver.available()==0);
 buf[i]=(char)IO_Driver.read();
  if (char(buf[i])=='\r')
    {
      buf[i]=0;
      break;
    }  
   }
}



void decode_serial(char *p){
      int i=0;
      while((str=strtok_r(p,",",&p))!=NULL)
        {
          vect[i]=atof(str);
          i++;
        }
      
      if(vect[0]==1023)//imu data
      {
           get_imu_gps(); 
      }
      if(vect[0]==1024)//gains data
      {
          get_Gains(); 
      }
      
      if(vect[0]==1025)
      {
       get_waypoints();  
      }
}

void get_waypoints(){
  int j=0;
  for(int i=0;i<=vect[1]-1;i++)
      {
        waypoint.lat[j]=vect[3*i+2];
        waypoint.lon[j]=vect[3*i+3];
        waypoint.alt[j]=vect[3*i+4];
        j++;
      }
      
      waypoint.length=vect[1];
      

    
        /*
  for(int i=7;i<=12;i++)
        waypoint.lon[i]=vect[i];
  
  for(int i=13;i<=18;i++)
        waypoint.alt[i]=vect[i];  
        */
  //Define home waypoint      
  waypoint.latHome=vect[2];
  waypoint.lonHome=vect[3];
  waypoint.altHome=vect[4];  
  waypoint.lat[waypoint.length]=vect[2];
  waypoint.lon[waypoint.length]=vect[3];
  waypoint.alt[waypoint.length]=vect[4];
  
}

void get_imu_gps(){
  imu.r=vect[1];
  imu.p=vect[2];
  imu.y=vect[3];
  gps.lat=vect[4];
  gps.lon=vect[5];
  gps.alt=vect[6];
  gps.gs=vect[7];
  gps.gc=vect[8];
  imu.ay=vect[9];
  command=vect[10];
  subcommand=vect[11];
  //roll, pitch, yaw, head, alt
  //Roll Gains
  /*
  kp[0]=vect[9]*1.0000;
  ki[0]=vect[10]*1.0000;
  kd[0]=vect[11]*1.0000;
  //Pitch Gains
  kp[1]=vect[12]*1.0000;
  ki[1]=vect[13]*1.0000;
  kd[1]=vect[14]*1.0000;
  //Altitude Gains
  kp[2]=vect[15]*1.0000;
  ki[2]=vect[16]*1.0000;
  kd[2]=vect[17]*1.0000;
  //Headinng Gains
  kp[3]=vect[18]*1.0000;
  ki[3]=vect[19]*1.0000;
  kd[3]=vect[20]*1.0000;
  //Yaw Gains
  kp[4]=vect[21]*1.0000;
  ki[4]=vect[22]*1.0000;
  kd[4]=vect[23]*1.0000;
  RefRoll=vect[24]/1000000;
*/
  
}

void get_Gains(){
  /*
  for(int i=1;i<=4;i++){
          kp[3*i-3]=vect[3*i-2]; //1 4 7 10 : 0 3 6 9
          ki[3*i-2]=vect[3*i-1]; //2 5 8 11 : 1 4 7 10
          kd[3*i-1]=vect[3*i];   //3 6 9 12 : 2 5 8 11  
    }
    */
}

void send_data(){ 
  
//  if (input_rc[5]>=1614){ //simulated fligh gear
    
     if(input_rc[4]>=1614){
       IO_Driver.print("surf");
         IO_Driver.print(',');
         IO_Driver.print("Manual");
         IO_Driver.print(',');
         IO_Driver.print(joinedwing.aileron);
         IO_Driver.print(',');
         IO_Driver.print(joinedwing.elevator);
         IO_Driver.print(',');
         IO_Driver.print(joinedwing.throttle);
         IO_Driver.print(',');
         IO_Driver.print(RefRoll);
         IO_Driver.print(',');
         IO_Driver.print(RefPitch);
         IO_Driver.print(',');
         IO_Driver.print(RefAlt);
         IO_Driver.print(',');
         IO_Driver.print(RefHead);
         IO_Driver.print(',');
         IO_Driver.print(HomeDistance);
         IO_Driver.print(',');                
         IO_Driver.print(NextWaypointDist);
         IO_Driver.print(',');                
         IO_Driver.print(joinedwing.rudder);
         IO_Driver.print(',');                
         IO_Driver.println(RefYaw);    
   
           
     }
     
        if(input_rc[4]<=1614 && input_rc[4]>=1214){
         IO_Driver.print("surf");
         IO_Driver.print(',');
         IO_Driver.print("Autopilot");
         IO_Driver.print(',');
         IO_Driver.print(joinedwing.aileron);
         IO_Driver.print(',');
         IO_Driver.print(joinedwing.elevator);
         IO_Driver.print(',');
         IO_Driver.print(joinedwing.throttle);
         IO_Driver.print(',');
         IO_Driver.print(RefRoll);
         IO_Driver.print(',');
         IO_Driver.print(RefPitch);
         IO_Driver.print(',');
         IO_Driver.print(RefAlt);
         IO_Driver.print(',');
         IO_Driver.print(RefHead);
         IO_Driver.print(',');
         IO_Driver.print(HomeDistance);
         IO_Driver.print(',');                
         IO_Driver.print(NextWaypointDist);
         IO_Driver.print(',');                
         IO_Driver.print(joinedwing.rudder);
         IO_Driver.print(',');                
         IO_Driver.println(RefYaw);    
    
          
         }
         
         
         
        
   if( input_rc[4]<=1114){
     

  // Serialprint("surf,UAV,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",joinedwing.aileron,joinedwing.elevator,joinedwing.throttle,RefRoll,RefPitch,RefAlt,RefHead,HomeDistance,NextWaypointDist,joinedwing.rudder,RefYaw);    
         IO_Driver.print("surf");        
         IO_Driver.print(',');                 
         IO_Driver.print("UAV");                  
         IO_Driver.print(',');                 
         IO_Driver.print(joinedwing.aileron);                  
         IO_Driver.print(',');                 
         IO_Driver.print(joinedwing.elevator);                  
         IO_Driver.print(',');                  
         IO_Driver.print(joinedwing.throttle);                 
         IO_Driver.print(',');                
         IO_Driver.print(RefRoll);                  
         IO_Driver.print(',');                
         IO_Driver.print(RefPitch);                 
         IO_Driver.print(',');          
         IO_Driver.print(RefAlt);                 
         IO_Driver.print(',');             
         IO_Driver.print(RefHead);               
         IO_Driver.print(',');              
         IO_Driver.print(HomeDistance);                 
         IO_Driver.print(',');                                 
         IO_Driver.print(NextWaypointDist);                 
         IO_Driver.print(',');                                  
         IO_Driver.print(joinedwing.rudder);            
         IO_Driver.print(',');                            
        IO_Driver.println(RefYaw);    
                 

     
        
       
           }
           /*
        else{
         IO_Driver.print("surf");  //0
         IO_Driver.print(',');
         IO_Driver.print("UAV");  //1
         IO_Driver.print(',');
         IO_Driver.print(joinedwing.aileron); //2
         IO_Driver.print(',');
         IO_Driver.print(joinedwing.elevator); //3
         IO_Driver.print(',');
         IO_Driver.print(joinedwing.throttle); //4
         IO_Driver.print(',');
         IO_Driver.print(RefRoll); //5
         IO_Driver.print(',');
         IO_Driver.print(RefPitch); //6
         IO_Driver.print(',');
         IO_Driver.print(RefAlt); //7
         IO_Driver.print(',');
         IO_Driver.print(RefHead); //8
         IO_Driver.print(',');                
         IO_Driver.print(HomeDistance); //9
         IO_Driver.print(',');                
         IO_Driver.print(NextWaypointDist); //10
         IO_Driver.print(',');                
         IO_Driver.print(joinedwing.rudder); //11
         IO_Driver.print(',');                
         IO_Driver.println(RefYaw);  //12 
         
        
      }*/
        
 /* 
}

     if(input_rc[5]<=1114){ //real uav
         IO_Driver.print("imu");
         IO_Driver.print(',');
         IO_Driver.print(imu.r);
         IO_Driver.print(',');
         IO_Driver.print(imu.p);
         IO_Driver.print(',');
         IO_Driver.print(imu.y);
         IO_Driver.print(',');
         IO_Driver.print(gps.lat);
         IO_Driver.print(',');
         IO_Driver.print(gps.lon);
         IO_Driver.print(',');
         IO_Driver.print(gps.alt);
         IO_Driver.print(',');
         IO_Driver.print(gps.gc);
         IO_Driver.print(',');
         IO_Driver.print(gps.gs);
         IO_Driver.print(',');
         IO_Driver.print(RefRoll);
         IO_Driver.print(',');
         IO_Driver.print(RefPitch);
         IO_Driver.print(',');
         IO_Driver.print(RefAlt);
         IO_Driver.print(',');
         IO_Driver.println(RefHead);
         IO_Driver.flush();
         delay(100);
     }
     
     */
}
