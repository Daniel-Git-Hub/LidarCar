#ifndef CAR_MOTOR_H_
#define CAR_MOTOR_H_

int car_init(int);
int car_move(double speedDouble, double angle);
int car_rotate(double);
int car_update();
int car_close();

#endif //CAR_MOTOR_H_