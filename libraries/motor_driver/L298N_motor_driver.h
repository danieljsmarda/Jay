/**
 * @file L298N_motor_driver.h
 * @brief Motor device driver for the L298N motor shield.
 * @author Miguel Grinberg and Matthew McDermott
 */

#include "motor_driver.h"

namespace Jay
{
    class Motor : public MotorDriver
    {
    public:

        Motor(int EN, int IN1, int IN2)
            : MotorDriver(), motor(EN, IN1, IN2), currentSpeed(0)
        {
        }

        void setSpeed(int speed)
        {
            currentSpeed = speed;
            if (speed >= 0) {
                motor.setSpeed(speed);
                motor.run(L298N::FORWARD);
            }
            else {
                motor.setSpeed(-speed);
                motor.run(L298N::BACKWARD);
            }
        }

        int getSpeed() const
        {
            return currentSpeed;
        }

    private:
        L298N motor;
        int currentSpeed;
    };
};