/*
Author: Faizal Patel
Lisbon 2011
*/
double P_last[3];
double Q[]={0.3,0.3,0.2};
double R[]={1.1,1.1,1};
double K[3];
double  x_temp_est[3];
double P_temp[3];
double x_est[3];
double P[3];
double kalman(double rate, double z_real,double dt,int s){
  //Prediction
  x_temp_est[s]+=rate*dt; //gyro angle
  P_temp[s]=P_last[s]+Q[s];
  //Kalman Gain
  K[s]=P_temp[s]+(1.0/(P_temp[s]+R[s]));
  //Correct
  x_est[s]=x_temp_est[s]+K[s]*(z_real-x_temp_est[s]);
  P[s]=(1-K[s])*P_temp[s];
  //new system
  P_last[0]=P[s];
  return x_est[s];
}
