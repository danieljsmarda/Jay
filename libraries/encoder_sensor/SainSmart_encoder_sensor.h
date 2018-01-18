/**

 * @brief Quadrature Encoder sensor driver for SainSmart motor.
 * @author Matthew McDermott
 */

#include "encoder_sensor.h"

namespace Jay
{
    class EncoderSensor : public EncoderSensorDriver
    {
    public:
        EncoderSensor(int pinA, int pinB)
            : EncoderSensorDriver(), 
              sensor(pinA, pinB)
        {
        }

        virtual long readEncoder()
        {
            long newPosition = sensor.read();
            return newPosition;

        }
    private:
        Encoder sensor;
    };
};