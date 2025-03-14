#include "esp_log.h"

#include <stdbool.h>
#include <math.h>

#include "ntc.h"
#include "ntc_data.h"

#define EPSILON 1e-4

static int float_equal(float a, float b)
{
    return fabs(a - b) < EPSILON;
}

static float find_temperature(float r, const ntc_breakpoint_t breakpoints[NTC_BREAKPOINTS])
{
    if (r >= breakpoints[0].r)
        return breakpoints[0].temperature;
    
    if (r <= breakpoints[NTC_BREAKPOINTS - 1].r) {
        return breakpoints[NTC_BREAKPOINTS - 1].temperature;
    }

    int left = 0, right = NTC_BREAKPOINTS - 1;
    while (left < right - 1) {
        int mid = left + (right - left) / 2;
        if (float_equal(breakpoints[mid].r, r)) {
            return breakpoints[mid].temperature;
        } else if (breakpoints[mid].r > r) {
            left = mid;
        } else {
            right = mid;
        }
    }

    float t0 = breakpoints[left].temperature;
    float t1 = breakpoints[right].temperature;
    float r0 = breakpoints[left].r;
    float r1 = breakpoints[right].r;

    float t = t0 + (r - r0) * (t1 - t0) / (r1 - r0);

    return t;
}

static float get_r_of_ntc(float vref, float r_s, float v_ntc)
{
    return (r_s * v_ntc) / (vref - v_ntc);
}

static float ntc_get_temperature(float vref, float r_s, float v_ntc, const ntc_breakpoint_t ntc[NTC_BREAKPOINTS])
{
    float r_ntc = get_r_of_ntc(vref, r_s, v_ntc);
    float t = find_temperature(r_ntc, ntc);

    return t;
}

float ntc_pcb_get_temperature(float vref, float r_s, float v_ntc)
{
    return ntc_get_temperature(vref, r_s, v_ntc, ntc_pcb);
}

float ntc_psu_get_temperature(float vref, float r_s, float v_ntc)
{
    return ntc_get_temperature(vref, r_s, v_ntc, ntc_psu);
}
