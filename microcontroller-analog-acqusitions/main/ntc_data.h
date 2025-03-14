#ifndef NTC_DATA_H
#define NTC_DATA_H

#define NTC_BREAKPOINTS     1451

struct ntc_breakpoint
{
    float temperature;
    float r;
};

typedef struct ntc_breakpoint ntc_breakpoint_t;

extern const ntc_breakpoint_t ntc_psu[NTC_BREAKPOINTS];

extern const ntc_breakpoint_t ntc_pcb[NTC_BREAKPOINTS];

#endif
