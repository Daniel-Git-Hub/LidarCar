#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

class motor_controller
{
    private:

    int pinP;
    int pinM;
    int pinEn;

    int pigPio;

    double lastSpeed;
    int pwm;

    public:

    motor_controller() {};
    motor_controller(int p, int m, int en);
    int set_pin(int p, int m, int en);
    int init(int p);

    int set_speed(double d);
    int set_pwm(int p);
    int close();

};


#endif