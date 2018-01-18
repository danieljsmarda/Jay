/**
 * @file encoder_sensor.h
 * @brief Encoder device driver definition for the Jay robot
 * @author Matthew McDermott
 */

namespace Jay
{
    class EncoderSensorDriver
    {
    public:

        virtual long readEncoder() = 0;
      
    };
};