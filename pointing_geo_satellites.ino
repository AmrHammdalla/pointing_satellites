// ----- Libraries
#include "I2Cdev.h"
#include "MPU9250.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <math.h>
#include "CytronMotorDriver.h"
#include <PID_v1.h>
#include <LiquidCrystal_I2C.h>                                          // YwRobot Arduino LCM1602 IIC V1 library                                       // YwRobot Arduino LCM1602 IIC V1 library

//MPU9250
MPU9250 accelgyro;
I2Cdev   I2C_M;

// vvvvvvvvvvvvvvvvvv  VERY VERY IMPORTANT vvvvvvvvvvvvvvvvvvvvvvvvvvvvv
//These are the previously determined offsets and scale factors for accelerometer and magnetometer, using MPU9250_cal and Magneto 1.2
//The AHRS will NOT work well or at all if these are not correct
//
// redetermined 12/16/2020
//acel offsets and correction matrix
  float A_B[3]{  723.48,  529.34,-2894.69};

 float A_Ainv[3][3]
  {{  0.61165, -0.01198,  0.00132},
  { -0.01198,  0.62639, -0.00629},
  {  0.00132, -0.00629,  0.60670}};

// mag offsets and correction matrix
   float M_B[3]
 {   18.27,   31.47,    5.47};

 float M_Ainv[3][3]
  {{  1.67867, -0.03948,  0.01149},
  { -0.03948,  1.63095, -0.07133},
  {  0.01149, -0.07133,  1.95078}};
  
  float G_off[3] = {544.2, -82.9, 58.3}; //raw offsets, determined for gyro at rest
// ^^^^^^^^^^^^^^^^^^^ VERY VERY IMPORTANT ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
char s[60]; //snprintf buffer
//raw data and scaled as vector
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
float Axyz[3];
float Gxyz[3];
float Mxyz[3];
#define gscale (250./32768.0)*(PI/180.0)  //gyro default 250 LSB per d/s -> rad/s

// NOW USING MAHONY FILTER

// These are the free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
// with MPU-9250, angles start oscillating at Kp=40. Ki does not seem to help and is not required.
#define kp 25.0
#define ki 0.0

// globals for AHRS loop timing

unsigned long now = 0, last = 0; //micros() timers
float deltat = 0;  //loop time in seconds
unsigned long now_ms, last_ms = 0; //millis() timers
unsigned long print_ms = 1000; //print every "print_ms" milliseconds


// Vector to hold quaternion
static float q[4] = {1.0, 0.0, 0.0, 0.0};
static float yaw, pitch, roll; //Euler angle output

// ----- Flags
bool Flag = false;
//-------variables
float   Declination = +4.76;                                             //  Degrees ... replace this declination with yours
double desiredAzimuth;  
double desiredElevation;
const float pi = 3.14159267;
float lng=-1,lat=-1,alt=-1,G,L,G_rad,L_rad;

//------ GPS init
  TinyGPSPlus gps;
  int RXPin = 8, TXPin=9, GPSBaud=9600;
  bool result=false;
  SoftwareSerial gpsSerial(RXPin, TXPin);
//---------------------------------------------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------//
void setup()
{
// join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Serial.begin(115200);
  while(!Serial); //wait for connection

  // initialize device
  accelgyro.initialize();
  // verify connection
  Serial.println(accelgyro.testConnection() ? "MPU9250 OK" : "MPU9250 ??");
  //-----------------------------GPS serial start---------------------------//
  gpsSerial.begin(GPSBaud);  
  while(!result){
    result=gps_init();
  }
  //----------------------------------------------------------------------------//          
  get_set_points();
  // desiredAzimuth=172.17;
  // desiredElevation=54.6;
  point_elevation(20);
  point_azimuth(desiredAzimuth); 
  point_elevation(desiredElevation);
}
//---------------------------------------------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------//
void loop(){  
};
//-------------------------------------------------------Functions-------------------------------------------------------------//
//-------------------------------------------------------Functions-------------------------------------------------------------//
//-------------------------------------------------------Functions-------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------------------//
  // bool gps_init(){  
  //     while (gpsSerial.available() > 0 && lng==-1 && lat==-1 && alt==-1)
  //     {  
  //       if (gps.encode(gpsSerial.read()))
  //       {
  //         if (gps.location.isValid())
  //         {
  //           lat= gps.location.lat();  
  //           lng=gps.location.lng();
  //           alt=gps.altitude.meters();

  //           Serial.print("Latitude: ");
  //           Serial.println(gps.location.lat(), 6);
  //           Serial.print("Longitude: ");
  //           Serial.println(gps.location.lng(), 6);
  //           Serial.print("Altitude: ");
  //           Serial.println(gps.altitude.meters());
  //           return true;
  //         }
  //         else
  //         {
  //           Serial.println(gps.location.lat());
  //           Serial.println("Location: Not Available");
  //           return false;
  //         }        
  //       }
  //       else
  //         return false;
  //     }
    
  //     if (millis() > 5000 && gps.charsProcessed() < 10)
  //     {
  //       Serial.println(F("No GPS data received: check wiring"));
  //       while(true);
  //     } 
  //     return false;
  // }
//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//
// void displayInfo()
// {
//   if (gps.location.isValid())
//   {
//     Serial.print("Latitude: ");
//     Serial.println(gps.location.lat(), 6);
//     Serial.print("Longitude: ");
//     Serial.println(gps.location.lng(), 6);
//     Serial.print("Altitude: ");
//     Serial.println(gps.altitude.meters());
//   }
//   else
//   {
//     Serial.println("Location: Not Available");
//   }
  
//   Serial.print("Date: ");
//   if (gps.date.isValid())
//   {
//     Serial.print(gps.date.month());
//     Serial.print("/");
//     Serial.print(gps.date.day());
//     Serial.print("/");
//     Serial.println(gps.date.year());
//   }
//   else
//   {
//     Serial.println("Not Available");
//   }

//   Serial.print("Time: ");
//   if (gps.time.isValid())
//   {
//     if (gps.time.hour() < 10) Serial.print(F("0"));
//     Serial.print(gps.time.hour());
//     Serial.print(":");
//     if (gps.time.minute() < 10) Serial.print(F("0"));
//     Serial.print(gps.time.minute());
//     Serial.print(":");
//     if (gps.time.second() < 10) Serial.print(F("0"));
//     Serial.print(gps.time.second());
//     Serial.print(".");
//     if (gps.time.centisecond() < 10) Serial.print(F("0"));
//     Serial.println(gps.time.centisecond());
//   }
//   else
//   {
//     Serial.println("Not Available");
//   }

//   Serial.println();
//   Serial.println();
//   delay(1000);
// }
//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//
  void get_set_points(){
      G=35.5-lng;  //replace 35.5 with the satellite longitude
      G_rad=G*DEG_TO_RAD;
      L=0+lat;
      L_rad=L*DEG_TO_RAD;
      if(lat>0) {
        //top right hemisphere
        if(G>=0){
          desiredAzimuth=180-atan(tan(G_rad)/sin(L_rad))*RAD_TO_DEG;
        }
        //top left hemisphere
        else if(G<0){
        desiredAzimuth=180+atan(tan(abs(G_rad))/sin(L_rad))*RAD_TO_DEG;    
        }
      } 
      else if(lat<0){
        //bottom right hemisphere
        if(G>0){
          desiredAzimuth=atan(tan(G_rad)/sin(abs(L_rad)))*RAD_TO_DEG;      
        }  
        //bottom left hemisphere  
        else if(G<0){
        desiredAzimuth=360-atan(tan(abs(G_rad))/sin(abs(L_rad)))*RAD_TO_DEG; 
        }  
      }
      float num=cos(G_rad)*cos(L_rad)-0.1512;
      float dem=sqrt(1-pow(cos(G_rad),2)*pow(cos(L_rad),2));   
      desiredElevation =atan(num/dem)*RAD_TO_DEG;      
  }
//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//
  void point_azimuth(double target){
    //----variables
      float azimuth;
      bool azimuthPointed=false;
    //------- azimuth motor directon  
      double azimuthMotorDir=0;
      double azimuthRestoration=target;
    //------- azimuth motor PWM  
      CytronMD azimuthMotor(PWM_DIR, 3, 4);  // PWM = Pin 3, DIR = Pin 4.
    //------- PID init    
      int modifiedAzimuthOutput;
      double Setpoint=azimuthRestoration, azimuthInput, azimuthOutput;
      double Kp=1.25, Ki=0.11, Kd=1.26; // kp=1, ki=0.14,  kd=1.25
      PID azimuthPID(&azimuthInput, &azimuthOutput, &Setpoint, Kp, Ki, Kd, DIRECT);
      azimuthPID.SetMode(AUTOMATIC);  
    while(!azimuthPointed){
     get_MPU_scaled();
      now = micros();
      deltat = (now - last) * 1.0e-6; //seconds since last update
      last = now;
      MahonyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2],Mxyz[1], Mxyz[0], -Mxyz[2], deltat);
      roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
      pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
      yaw   = atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - ( q[2] * q[2] + q[3] * q[3]));
      // to degrees
      yaw   *= 180.0 / PI;
      pitch *= 180.0 / PI;
      roll *= 180.0 / PI;

      yaw=-yaw+Declination;
      if(yaw<0) yaw += 360.0;
      if(yaw>360.0) yaw -= 360.0;
      
      azimuth=yaw;

      //sensor leading azimuthRestoration -> rotate anti-clockwise
        if(azimuth-azimuthRestoration>0.05){
          Setpoint=azimuth;
          azimuthInput=azimuthRestoration;
          azimuthMotorDir=-1;
        }  
      //azimuthRestoration leading sensor -> rotate clock-wise
        else if(azimuthRestoration-azimuth>0.05){
          Setpoint=azimuthRestoration;
          azimuthInput=azimuth;
          azimuthMotorDir=1;
        }
        else{
          Setpoint=0;
          azimuthInput=0;
          azimuthMotorDir=0;   
          azimuthPointed=true;
        }; 
      azimuthPID.Compute();
      if(azimuthOutput>=255){
        modifiedAzimuthOutput=255;
      }else if(azimuthOutput<=0){
        modifiedAzimuthOutput=0;
      }else{
        modifiedAzimuthOutput=azimuthOutput;
      } 
      azimuthMotor.setSpeed(azimuthMotorDir*modifiedAzimuthOutput);  
      Serial.print("azimuth:");
      Serial.print(azimuth);
      Serial.println();
      Serial.print("modified_azimuth_output:");
      Serial.print(modifiedAzimuthOutput);
      Serial.println();
    }
    Serial.println("Azimuth pointed successfully!") ;
    Serial.println("Elevation pointing will start in 5 seconds....") ;
    delay(5000);
  }
//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//
void point_elevation(double target){
  //----variables 
  float elevation; 
  bool restored=false;
  //------- elevation motor directon  
  double elevationMotorDir=0;
  double elevationRestoration=target;
  //------- elevation motor PWM  
  CytronMD elevationMotor(PWM_DIR, 6, 7);  // PWM = Pin 1, DIR = Pin 2.
  //------- PID init
  int modifiedElevationOutput;
  double setPoint=elevationRestoration, elevationInput, elevationOutput;
  double Kp=1.25, Ki=1.15, Kd=1.26;  //ki=0.2
  PID elevationPID(&elevationInput, &elevationOutput, &setPoint, Kp, Ki, Kd, DIRECT);  
  elevationPID.SetMode(AUTOMATIC); 
  while(!restored){  
    Serial.print("elevation_target: ") ;
    Serial.print(target);
    Serial.println();  
    Serial.println("*******************");    
    get_MPU_scaled();
    now = micros();
    deltat = (now - last) * 1.0e-6; //seconds since last update
    last = now;
    MahonyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2],Mxyz[1], Mxyz[0], -Mxyz[2], deltat);
    pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
    pitch *= (-180.0 / PI);
    elevation=pitch-3.925; //biased by 3.925 degree //should be zeroed in next box //elevation is between 0->90 for looking to sky, -90->0 for looking inside earth
    //------determine direction-------//
      if (elevation-elevationRestoration>0.5){
        setPoint=elevation;
        elevationInput=elevationRestoration;        
        elevationMotorDir=-1;    //colud be swapped
      }
      else if(elevationRestoration-elevation>0.5){
        setPoint=elevationRestoration;
        elevationInput=elevation;
        elevationMotorDir=1;   //colud be swapped
      }  
      else{
        setPoint=0;
        elevationInput=0;
        elevationMotorDir=0;   
        restored=true;        
      }
    //----calculate elevationOutput
    elevationPID.Compute();
      if(elevationOutput>=255){
        modifiedElevationOutput=255;
      }else if(elevationOutput<=0){
        modifiedElevationOutput=0;
      }else{
        modifiedElevationOutput=elevationOutput;
      } 
      elevationMotor.setSpeed(elevationMotorDir*modifiedElevationOutput);  
      Serial.print("modified_elevation_output:");
      Serial.print(modifiedElevationOutput);
      Serial.println();
      Serial.print("Direction:");
      Serial.print(elevationMotorDir); 
      Serial.println();
      Serial.print("elevation:");
      Serial.print(elevation);
      Serial.println();
    }
    if(target==20){
      Serial.println("Elevation Restored Successfully!") ;
      Serial.println("Azimuth pointing will start in 5 seconds....") ;
      delay(5000);
    }else{
      Serial.println("Pointed Successfully!") ;      
    }
}   
//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//
void get_MPU_scaled(void) {
  float temp[3];
  int i;
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

  Gxyz[0] = ((float) gx - G_off[0]) * gscale; //250 LSB(d/s) default to radians/s
  Gxyz[1] = ((float) gy - G_off[1]) * gscale;
  Gxyz[2] = ((float) gz - G_off[2]) * gscale;

  Axyz[0] = (float) ax;
  Axyz[1] = (float) ay;
  Axyz[2] = (float) az;
  //apply offsets (bias) and scale factors from Magneto
  for (i = 0; i < 3; i++) temp[i] = (Axyz[i] - A_B[i]);
  Axyz[0] = A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2];
  Axyz[1] = A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2];
  Axyz[2] = A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2];
  vector_normalize(Axyz);

  Mxyz[0] = (float) mx;
  Mxyz[1] = (float) my;
  Mxyz[2] = (float) mz;
  //apply offsets and scale factors from Magneto
  for (i = 0; i < 3; i++) temp[i] = (Mxyz[i] - M_B[i]);
  Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
  vector_normalize(Mxyz);
}
//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//
// Mahony scheme uses proportional and integral filtering on
// the error between estimated reference vectors and measured ones.
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
  // Vector to hold integral error for Mahony method
  static float eInt[3] = {0.0, 0.0, 0.0};
  // short name local variable for readability
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;
  /*
    // already done in loop()

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // Handle NaN
    norm = 1.0f / norm;       // Use reciprocal for division
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // Handle NaN
    norm = 1.0f / norm;       // Use reciprocal for division
    mx *= norm;
    my *= norm;
    mz *= norm;
  */
  // Reference direction of Earth's magnetic field
  hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
  hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
  bx = sqrt((hx * hx) + (hy * hy));
  bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

  // Estimated direction of gravity and magnetic field
  vx = 2.0f * (q2q4 - q1q3);
  vy = 2.0f * (q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;
  wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
  wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
  wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

  // Error is cross product between estimated direction and measured direction of the reference vectors
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  if (ki > 0.0f)
  {
    eInt[0] += ex;      // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
    // Apply I feedback
    gx += ki*eInt[0];
    gy += ki*eInt[1];
    gz += ki*eInt[2];
  }


  // Apply P feedback
  gx = gx + kp * ex; 
  gy = gy + kp * ey;
  gz = gz + kp * ez;

  // Integrate rate of change of quaternion
 // small correction 1/11/2022, see https://github.com/kriswiner/MPU9250/issues/447
  gx = gx * (0.5*deltat); // pre-multiply common factors
  gy = gy * (0.5*deltat);
  gz = gz * (0.5*deltat);
  float qa = q1;
  float qb = q2;
  float qc = q3;
  q1 += (-qb * gx - qc * gy - q4 * gz);
  q2 += (qa * gx + qc * gz - q4 * gy);
  q3 += (qa * gy - qb * gz + q4 * gx);
  q4 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}
//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//
float vector_dot(float a[3], float b[3])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//
void vector_normalize(float a[3])
{
  float mag = sqrt(vector_dot(a, a));
  a[0] /= mag;
  a[1] /= mag;
  a[2] /= mag;
}
