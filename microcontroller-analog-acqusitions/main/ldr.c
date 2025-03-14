#include "ldr.h"

#include <math.h>

// Constants for NSL-6112 resistance-to-luminance conversion
#define A 1000000.0  // Empirical constant
#define B -1.5       // Empirical exponent

float r_to_luminance(float resistance)
{
    return A * pow(resistance, B);
}

static float get_r_of_ldr(float vref, float r_s, float v_ldr)
{
    return (r_s * v_ldr) / (vref - v_ldr);
}

float ldr_get_luminosity(float vref, float r_s, float v_ldr)
{
    float r_ldr = get_r_of_ldr(vref, r_s, v_ldr);
    float l = r_to_luminance(r_ldr);

    return l;
}