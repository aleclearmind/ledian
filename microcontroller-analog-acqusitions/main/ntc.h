#ifndef NTC_H
#define NTC_H

float ntc_pcb_get_temperature(float vref, float r_s, float v_ntc);

float ntc_psu_get_temperature(float vref, float r_s, float v_ntc);

#endif
