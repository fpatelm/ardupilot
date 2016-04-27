/*
Author: Faizal Patel
Lisbon 2011
*/
void init_rc_input(){
  pinMode(ch1,INPUT);
  pinMode(ch2,INPUT);
  pinMode(ch3,INPUT);
  pinMode(ch4,INPUT);
  pinMode(ch5,INPUT);
  pinMode(ch6,INPUT);
  pinMode(ch7,INPUT);
 
}

void read_rc_receiver(){


 /*
  for(int i=0;i<=19;i++)
  {
   val0+=
   val5+=
  }
  */

input_rc[0]=pulseIn(ch1,HIGH);
  
  input_rc[1]=pulseIn(ch2,HIGH);
  
  input_rc[2]=pulseIn(ch3,HIGH);
  
  input_rc[3]=pulseIn(ch4,HIGH);
  
  input_rc[4]=pulseIn(ch5,HIGH);
   
  input_rc[5]=pulseIn(ch6,HIGH);//val5/20;//=pulseIn(ch6,HIGH);
     input_rc[6]=pulseIn(ch7,HIGH);
   
  
}
