/*
Author: Faizal Patel
Lisbon 2011
*/
void move_servos(){
  //JoinedWing Configuration
    aileron_servos_jw();
    elevator_servos_jw();
    rudder_servos_jw();
  
}

void aileron_servos_ca(){
  double ang=30*joinedwing.aileron;
  C1_R.write(ang+90);
  C1_L.write(-ang+90);
}

void elevator_servos_ca(){
  double ang=30*joinedwing.elevator;
  C2_R.write(ang+90);
  C2_L.write(-ang+90);

}

void rudder_servos_ca(){
  double ang=30*joinedwing.rudder;
  C3_R.write(ang+90);
  C3_L.write(-ang+90);
}

void throttle_servos_ca(){
  
  
  
}

void aileron_servos_jw(){
  double ang=30*joinedwing.aileron;
  C2_R.write(ang+90);
  C3_R.write(ang+90);
  C2_L.write(-ang+90);
  C3_L.write(-ang+90);
}

void elevator_servos_jw(){
  double ang=30*joinedwing.elevator;
  C4_R.write(ang+90);
  C4_L.write(-ang+90);
}

void rudder_servos_jw(){
  double ang=30*joinedwing.rudder;
  C1_R.write(ang+90);
  C5_R.write(ang+90);
  C1_L.write(-ang+90);
  C5_L.write(-ang+90);
}

void throttle_servos_jw(){
  
  
  
}
void init_servos(){
  C1_R.attach(C1_p1);
  C2_R.attach(C2_p1);
  C3_R.attach(C3_p1);
  C4_R.attach(C4_p1);
  C5_R.attach(C5_p1);
  C1_L.attach(C1_p2);
  C2_L.attach(C2_p2);
  C3_L.attach(C3_p2);
  C4_L.attach(C4_p2);
  C5_L.attach(C5_p2);
  C6.attach(C6_p); //Engine Controller
  
}
