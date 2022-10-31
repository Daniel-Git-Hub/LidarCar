
#include <pigpiod_if2.h>

#include "rotary_encoder.h"

#include "board_def.h"

/*

             +---------+         +---------+      0
             |         |         |         |
   A         |         |         |         |
             |         |         |         |
   +---------+         +---------+         +----- 1

       +---------+         +---------+            0
       |         |         |         |
   B   |         |         |         |
       |         |         |         |
   ----+         +---------+         +---------+  1

*/

void static encoder_call(int pi, unsigned user_gpio, unsigned level, uint32_t tick, void * enc)
{
   motor_encoder * enc_class = (motor_encoder *)enc;

   enc_class->internal_call(user_gpio, level);

}


motor_encoder::motor_encoder(int ch1, int ch2){
   set_pin(ch1, ch2);
}

int motor_encoder::set_pin(int ch1, int ch2){
   enc_1 = ch1;
   enc_2 = ch2;

   return 0;
}

int motor_encoder::init(int pigPio){

   set_mode(pigPio, enc_1, PI_INPUT);
   set_mode(pigPio, enc_2, PI_INPUT);

   //No pull up for these rotary

   callback_ex(pigPio, enc_1, EITHER_EDGE, encoder_call, this);
   callback_ex(pigPio, enc_2, EITHER_EDGE, encoder_call, this);
   
   lastUpdateTime = time_time();
   
   return 0;
}

double motor_encoder::update(double sec){
   // double dt = sec - lastUpdateTime;
   double dt = sec;
   speed = (pos - lastPos) * M_PER_ENCODER / dt;
   lastPos = pos;
   lastUpdateTime = sec;

   return speed;
}

int motor_encoder::internal_call(int pin, int level){
   if(pin == enc_1){
      prev1 = level << 2;
   }else if(pin == enc_2){
      prev2 = level << 3;
   }else{
      return 1;
   }
   uint8_t tempState = prev1 | prev2 | (state & 0x03);
   state = tempState >> 2;
   switch(tempState){
      case 1: case 7: case 8: case 14:
         pos++;
         break;
      case 2: case 4: case 11: case 13:
         pos--;
         break;
      case 3: case 12:
         pos += 2;
         break;
      case 6: case 9:
         pos -= 2;
         break;
    }
    return 0;
}
