/*
Author: Faizal Patel
Lisbon 2011
*/
TinyGPS gps_driver;

void getgps(TinyGPS &gps_driver){
  float latitude, longitude,altitudez,curse,speeds;
  gps_driver.f_get_position(&latitude,&longitude);
  altitudez=gps_driver.f_altitude(); 
  curse=gps_driver.f_course();
  speeds=gps_driver.f_speed_kmph();
  gps.lat=latitude;
  gps.lon=longitude;
  gps.alt=altitudez;
  gps.gs=speeds;
  gps.gc=curse;
}

void seed_gps(){
  
  while(Serial1.available()){
    byte c=Serial1.read();
    if (gps_driver.encode(c)){
     getgps(gps_driver); 
     
      }
   }
  
}
