#ifndef __PANGOTERM_H__
#define __PANGOTERM_H__

#include "vterm.h"

#include <gtk/gtk.h>

typedef struct PangoTerm PangoTerm;

typedef size_t PangoTermWriteFn(const char *bytes, size_t len, void *user);
typedef void PangoTermResizedFn(int rows, int cols, void *user);

typedef struct {
  int cols;
  VTermScreenCell cells[];
} PangoTermScrollbackLine;

struct PangoTerm {
  VTerm *vt;
  VTermScreen *vts;

  GtkIMContext *im_context;

  int mousemode;

  GdkRectangle pending_area;
  /* Pending glyphs to flush in flush_pending */
  GString *glyphs;
  GArray *glyph_widths;
  /* Pending area to erase in flush_pending */
  int erase_columns;
  /* Is pending area DWL? */
  int pending_dwl;

  struct {
    struct {
      unsigned int bold      : 1;
      unsigned int underline : 2;
      unsigned int italic    : 1;
      unsigned int reverse   : 1;
      unsigned int strike    : 1;
      unsigned int font      : 4;
      unsigned int dwl       : 1;
      unsigned int dhl       : 2;
      unsigned int small     : 1;
      unsigned int baseline  : 2;
    } attrs;
    GdkColor fg_col;
    GdkColor bg_col;
    PangoAttrList *pangoattrs;
    PangoLayout *layout;
  } pen;

  int rows;
  int cols;

  int on_altscreen;
  int scroll_offs;

  int scroll_size;
  int scroll_current;
  PangoTermScrollbackLine **sb_buffer;

  PangoTermWriteFn *writefn;
  void *writefn_data;

  PangoTermResizedFn *resizedfn;
  void *resizedfn_data;

  int n_fonts;
  char **fonts;
  PangoFontDescription **fontdescs;
  int *fontoffsets;
  double *fontxscaling;
  double font_size;

  int cell_width_pango;
  int cell_width;
  int cell_height;

  GdkColor fg_col;
  GdkColor bg_col;

  int has_focus;
  int cursor_visible;    /* VTERM_PROP_CURSORVISIBLE */
  int cursor_blinkstate; /* during high state of blink */
  int cursor_hidden_for_redraw; /* true to temporarily hide during redraw */
  VTermPos cursorpos;
  VTermPos last_visible_cursorpos;
  GdkColor cursor_col;
  int cursor_shape;

#define CURSOR_ENABLED(pt) ((pt)->cursor_visible && !(pt)->cursor_hidden_for_redraw)

  guint cursor_timer_id;

  GtkWidget *termwin;

  cairo_surface_t *buffer;
  GdkWindow *termdraw;
  /* area in buffer that needs flushing to termdraw */
  GdkRectangle dirty_area;

  /* These four positions relate to the click/drag highlight state */

  enum { NO_DRAG, DRAG_PENDING, DRAGGING } dragging;
  /* Initial mouse position of selection drag */
  VTermPos drag_start;
  /* Current mouse position of selection drag */
  VTermPos drag_pos;

  /* Start and stop bounds of the selection */
  bool highlight_valid;
  VTermPos highlight_start;
  VTermPos highlight_stop;

  GtkClipboard *selection_primary;
  GtkClipboard *selection_clipboard;

  GString *outbuffer;
  GString *tmpbuffer; /* for handling VTermStringFragment */
};

PangoTerm *pangoterm_new(int rows, int cols);
void pangoterm_free(PangoTerm *pt);

guint32 pangoterm_get_windowid(PangoTerm *pt);

void pangoterm_set_default_colors(PangoTerm *pt, GdkColor *fg_col, GdkColor *bg_col);
void pangoterm_set_font_size(PangoTerm *pt, double size);
void pangoterm_set_fonts(PangoTerm *pt, char *font, char **alt_fonts); // ptr not value

void pangoterm_set_title(PangoTerm *pt, const char *title);

void pangoterm_start(PangoTerm *pt);

void pangoterm_begin_update(PangoTerm *pt);
void pangoterm_push_bytes(PangoTerm *pt, const char *bytes, size_t len);
void pangoterm_end_update(PangoTerm *pt);

void pangoterm_set_write_fn(PangoTerm *pt, PangoTermWriteFn *fn, void *user);

void pangoterm_set_resized_fn(PangoTerm *pt, PangoTermResizedFn *fn, void *user);

#endif
