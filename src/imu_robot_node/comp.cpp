
#include "comp.h"
#include <math.h>

double accRoll = 0;
double accPitch = 0;
double accYaw = 0;            // units degrees (roll and pitch noisy, yaw not possible)
double complementaryRoll = 0; 
double complementaryPitch = 0;
double complementaryYaw = 0;  // units degrees (excellent roll, pitch, yaw minor drift)

tf::Quaternion compHeading;


tf::Quaternion * comp_calc(double dt, imu_reading_t * reading){
  accRoll = atan2(reading->tranAccelY, reading->tranAccelZ);
  accPitch = atan2(-reading->tranAccelX, sqrt(reading->tranAccelY*reading->tranAccelY + reading->tranAccelX*reading->tranAccelX));

  double lastFrequency = dt;

  complementaryRoll  = complementaryRoll  + reading->tranGyroX * lastFrequency;
  complementaryPitch = complementaryPitch + reading->tranGyroY * lastFrequency;
  complementaryYaw   = complementaryYaw   + reading->tranGyroZ * lastFrequency;

  // complementaryRoll = 0.98*complementaryRoll + 0.02*accRoll;
  // complementaryPitch = 0.98*complementaryPitch + 0.02*accPitch;

  compHeading.setRPY(complementaryRoll, complementaryPitch, complementaryYaw);

  return &compHeading;
}

int comp_print(){
  // printf("Heading %lf %lf %lf\n", reading->tranGyroX, reading->tranGyroY, reading->tranGyroZ);
  printf("Heading %lf %lf %lf\n", complementaryRoll, complementaryPitch, complementaryYaw);
  return 0;
}