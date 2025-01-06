#pragma once

#include "vterm.h"

typedef struct PangoTerm PangoTerm;

void ledian_pangoterm_start(PangoTerm *pt);
void ledian_term_movecursor(PangoTerm *pt, VTermPos pos);
void ledian_term_damage(PangoTerm *pt, VTermRect rect);
void ledian_pangoterm_free(PangoTerm *pt);
void ledian_stream(void);
void ledian_selftest(void);
