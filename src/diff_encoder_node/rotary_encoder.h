#ifndef ROTARY_ENCODER_H
#define ROTARY_ENCODER_H

#include <stdint.h>


class motor_encoder
{

   private:

   int enc_1;
   int enc_2;
   
   uint8_t prev1;
   uint8_t prev2;
   uint8_t state;

   int lastPos;
   
   double lastUpdateTime;

   public:

   int pos;
   double speed;

   motor_encoder() {};
   motor_encoder(int ch1, int ch2);

   int init(int pigPio);
   int set_pin(int ch1, int ch2);

   int internal_call(int pin, int level);

   double update(double sec);

};

#endif
