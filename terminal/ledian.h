#pragma once

#include "vterm.h"

typedef struct PangoTerm PangoTerm;

void ledian_term_damage(VTermRect rect, PangoTerm *pt);
void ledian_pangoterm_start(PangoTerm *pt);
void ledian_pangoterm_free(PangoTerm *pt);
