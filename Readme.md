## Getting started
This project/sketch used to point your antenna to geo satellites.

make sure to calibrate the mpu9250, check https://github.com/jremington/MPU-9250-AHRS/tree/master, start from the instructions file inside docs folder to accurately calibrate your IMU.

don't forget to set the magnetic declination relative to your location, 4.76 in case of mine.

set the ki,kp,kd parameters for your PID controller for both loops (azimuth,elevation).

## Components

Arduino board (UNO) in my case, Cytron motor driver, Gps neo-6m, mpu9250 (IMU), 2 DC Motors (elevation,azimuth)





