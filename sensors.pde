/*
Author: Faizal Patel
Lisbon 2011
*/
void calc_gyro_angles(long delta_t){
  angleGy+=rateGy*delta_t/1000;
  angleGx+=rateGx*delta_t/1000;
  angleGz+=rateGz*delta_t/1000;
  if (angleGz>360)
        angleGz-=360;
  if (angleGz<0)
        angleGz+=360;   
}
void calc_rates_values(){
  rateGy=(adGy-adGy0)/1.0323;
  rateGx=(adGx-adGx0)/1.0323;
  rateGz=(adGz-adGz0)/1.0323;
  valAz=(adAz-adAz0)/102.3;
  valAy=(adAy-adAy0)/102.3;
  valAx=(adAx-adAx0)/102.3;
}

void calc_yaw(long delta_t){
  yaw_angle=yaw_angle + delta_t*imu.y;
}
  
void read_analog_pins(){
  adGy=analogRead(A0);
  adGx=analogRead(A1);
  adGz=analogRead(A2);
  adAz=analogRead(A3);
  adAy=analogRead(A4);
  adAx=analogRead(A5);   
  delay(17);
}

void calc_acle_angles(){
  double R;
  R=sqrt(valAx*valAx+valAy*valAy+valAz*valAz);
  angleAx=ToDeg(atan2(valAy,valAx))-50;
  angleAy=ToDeg(atan2(valAx,sqrt((valAy*valAy)+(valAx*valAx))));
  angleAx=acos(valAx/R)*57.295779513082320876798154814105;
  //angleAy=acos(valAy/R)*57.295779513082320876798154814105;
  angleAz=acos(valAz/R)*57.295779513082320876798154814105-60;
}


void imu_func(double delta_t){
    read_analog_pins();
    calc_rates_values();
    calc_acle_angles();
    calc_gyro_angles(delta_t);
    
    if(input_rc[4]==1314)
    {
     RefRoll=-kalman(rateGx,map(adAx-adAx0,-105,105,-90,90),delta_t/1000,0);
     RefRoll=constrain(RefRoll,-90,90)/90;
     RefPitch=kalman(rateGy,map(adAy-adAy0,-105,105,-90,90),delta_t/1000,1);
     RefPitch=constrain(RefPitch,-90,90)/90;
     RefYaw=kalman(rateGz,angleGz,delta_t/1000,2);
     //rateGy*sin(RefRoll)/cos(RefPitch)+rateGz*cos(RefPitch)/cos(RefRoll)*delta_t+RefYaw;
    }
    
    
    imu.r=-kalman(rateGx,map(adAx-adAx0,-105,105,-90,90),delta_t/1000,0);//kalman_filter(rateGx,map(adAx-adAx0,-105,105,-90,90),delta_t/1000,0);
    imu.r=constrain(imu.r,-90,90);
    imu.p=kalman(rateGy,map(adAy-adAy0,-105,105,-90,90),delta_t/1000,1);//=kalman_filter((long)rateGy,(long)angleAy,(double)delta_t/1000,1);
    imu.p=constrain(imu.p,-90,90);
    //yaw_angg=rateGy*sin(imu.r)/cos(imu.p)+rateGz*cos(imu.p)/cos(imu.r)*delta_t+yaw_angg;
    imu.y=angleGz;
    
    
    

   //RefYaw=kalman(rateGz,map(adAz-adAz0,-105,105,-180,180),delta_t/1000,2);//kalman_filter((long)rateGz,(long)angleAz,(double)delta_t/1000,2); 
  
}

void init_var(){
  
  roll=0;
  pitch=0;
  yaw=0;
}

void init_analog_ports(){
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);
  pinMode(A4,INPUT);
  pinMode(A5,INPUT);
  pinMode(A6,INPUT);
  pinMode(A7,INPUT);
  delay (100); 
 
}


void init_g_a(){
 adGz0=analogRead(A2);
 delay(3);
 adGx0=analogRead(A1);
 delay(3);
 adGy0=analogRead(A0);
 delay(3);
 adAz0=analogRead(A3);
 delay(3);
 adAy0=analogRead(A4);
 delay(3);
 adAx0=analogRead(A5);
 delay(26);
}
