#include "current_sensor.h"

/* ACS71240KEXBLT-010B3 */

#define CURRENT_SENSITIVITY     0.132f
#define ZERO_POINT              1.65f

float current_sensor_get_value(float v)
{
    float a_v = v - ZERO_POINT;
    float a = a_v / CURRENT_SENSITIVITY;
    
    return -a;
}