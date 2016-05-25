# ardupilot
Arduino Auto Pilot System

Autopilot engine for arduino.

Communication channel is serial.

We should input 3 types of data to initialize the autopilot:

KID control gains:
1024,

Waypoints:
1025, lat1,lon1,alt1,...,latn,lonn,altn

IMU and GPS:
1023,r,p,q,alt,lon,alt,gs,gc,ay,command,subcommand


Output string: 
surf,UAV,aileron,elevator,throtle, refRoll, refPitch,refAlt,refHead,HomeDistance,NextWayPointDist,rudder,refYAw
