#define _GNU_SOURCE
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "conf.h"
#include "ledian.h"
#include "pangoterm.h"

CONF_STRING(tty_path, 0, "/dev/ttyUSB0", "Path to the TTY", "PATH");

uint8_t *ScreenBuffer = NULL;
VTermPos ScreenCursor = {0, 0};
FILE *output_file = NULL;

static int set_interface_attribs(FILE *file, int speed, int parity) {
  int fd = fileno(file);
  struct termios tty;
  if (tcgetattr(fd, &tty) != 0) {
    fprintf(stderr, "error %d from tcgetattr\n", errno);
  }

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  tty.c_iflag &= ~IGNBRK; // disable break processing
  tty.c_lflag = 0;        // no signaling chars, no echo,
  // no canonical processing
  tty.c_oflag = 0;     // no remapping, no delays
  tty.c_cc[VMIN] = 0;  // read doesn't block
  tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

  tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls,
  // enable reading
  tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
  tty.c_cflag |= parity;
  tty.c_cflag &= ~CSTOPB;
  // tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    fprintf(stderr, "error %d from tcsetattr\n", errno);
  }

  return 0;
}

static void set_blocking(FILE *file, int should_block) {
  struct termios tty;
  int fd = fileno(file);
  memset(&tty, 0, sizeof tty);
  if (tcgetattr(fd, &tty) != 0) {
    fprintf(stderr, "error %d from tggetattr\n", errno);
  }

  tty.c_cc[VMIN] = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

  if (tcsetattr(fd, TCSANOW, &tty) != 0)
    fprintf(stderr, "error %d setting term attributes\n", errno);
}

static void wait_for_ack(FILE *file) {
#if 0
#define BUFFER_SIZE (1024 * 1024 * 2)
  uint8_t Buffer[BUFFER_SIZE];
  uint8_t *BufferPointer = &Buffer[0];
  void *Result = NULL;
  struct pollfd Query = {file, POLLIN, 0};
  while (Result == NULL || poll(&Query, 1, 0) == 1) {
    size_t Length = fread(file, BufferPointer, 1024);
    fwrite(BufferPointer, 1, Length, stderr);
    BufferPointer += Length;
    Result = memmem(&Buffer[0], BufferPointer - &Buffer[0], "ACK\n", 4);
  }
  printf("Got ACK\n");
#endif
}

static void strict_write(FILE *file, const void *buf, size_t count) {
  ssize_t Result = fwrite(buf, 1, count, file);
  assert(Result == count);
}

typedef uint8_t ledian_id_t;
typedef uint32_t length_t;

static void send_message(FILE *file, ledian_id_t ID, uint8_t *Data,
                         length_t Size) {
  strict_write(file, &ID, 1);
  strict_write(file, &Size, 4);
  strict_write(file, Data, Size);

  fflush(file);

  wait_for_ack(file);
}

typedef struct {
  uint8_t H, S, V;
} HSV;

HSV ColorMap[] = {
#include "colors.inc"
};

/// Send a specific cell to file
static void send_cell(FILE *file, PangoTerm *pt, int Line, int Column) {
  uint8_t Value = *(ScreenBuffer + Line * pt->cols + Column);
  bool IsCursor = Line == ScreenCursor.row && Column == ScreenCursor.col;
#ifdef VERBOSE
  printf("%.2x%.2x%.2x%.2x ", Value, 0xFF, 0x30, IsCursor ? 0x01 : 0x00);
#endif

  assert(sizeof(ColorMap) == 256 * 3);

  HSV Color = ColorMap[Value];

#if 0
  Color.H = Value;
  Color.S = 0xFF;
  Color.V = (Value == 0 || Value == ' ') ? 0 : 0x30;
#endif

  uint8_t Buffer[4] = {Color.H, Color.S, Color.V, IsCursor ? 0x01 : 0x00};
  strict_write(file, &Buffer, 4);
}

/// Send a rectangle
static void send_rectangle(FILE *file, const VTermRect *rect, PangoTerm *pt) {
  assert(file != NULL);
#ifdef VERBOSE
  printf("\e[1;1H\e[2J");

  uint8_t *CursorPointer =
      ScreenBuffer + ScreenCursor.row * pt->cols + ScreenCursor.col;
  uint8_t UnderCursor = *CursorPointer;
  *(CursorPointer) = '*';
  for (size_t Line = 0; Line < pt->rows; ++Line) {
    fwrite(ScreenBuffer + Line * pt->cols, pt->cols, 1, stdout);
    printf("\n");
  }
  *(CursorPointer) = UnderCursor;
#endif

  typedef struct {
    uint32_t Column;
    uint32_t Line;
  } cursor_t;
  g_assert(sizeof(cursor_t) == 8);

  for (size_t Line = rect->start_row; Line < rect->end_row; ++Line) {
    // Move cursor to new line
    cursor_t Cursor = {rect->start_col, Line};
    send_message(file, 3, (uint8_t *)&Cursor, sizeof(Cursor));

    length_t Length = 4 * (rect->end_col - rect->start_col);
    strict_write(file, "\x02", 1);
    strict_write(file, &Length, 4);
    for (size_t Column = rect->start_col; Column < rect->end_col; ++Column) {
      send_cell(file, pt, Line, Column);
    }
    wait_for_ack(file);
  }
}

void ledian_term_damage(VTermRect rect, PangoTerm *pt) {
#ifdef VERBOSE
  fprintf(stderr, "term_damage({ %d, %d, %d, %d });\n", rect.start_row,
          rect.end_row, rect.start_col, rect.end_col);
#endif

  // Update the ScreenBuffer
  for (size_t Line = rect.start_row; Line < rect.end_row; ++Line) {
    for (size_t Column = rect.start_col; Column < rect.end_col; ++Column) {
      VTermPos CurrentPosition = {Line, Column};
      VTermScreenCell CurrentCell;
      vterm_screen_get_cell(pt->vts, CurrentPosition, &CurrentCell);
      char NewChar = CurrentCell.chars[0];
      *(ScreenBuffer + Line * pt->cols + Column) = NewChar;
    }
  }

  // Flush the changed area
  send_rectangle(output_file, &rect, pt);
}

void ledian_pangoterm_start(PangoTerm *pt) {
  //
  // Resources cleanup
  //
  ledian_pangoterm_free(pt);

  //
  // Allocate and initialize screen buffer
  //
  const size_t ScreenBufferSize = pt->cols * pt->rows;
  assert(ScreenBuffer == NULL);
  ScreenBuffer = malloc(ScreenBufferSize);
  memset(ScreenBuffer, ' ', ScreenBufferSize);

  //
  // Open TTY
  //
  assert(output_file == NULL);

  // Check if file exists
  if (access(CONF_tty_path, F_OK) != 0) {
    fprintf(stderr, "%s does not exist.\n", CONF_tty_path);
    exit(1);
  }

  // Check if the file is writeable
  if (access(CONF_tty_path, W_OK) != 0) {
    fprintf(stderr, "%s is not writeable.\n", CONF_tty_path);
    exit(1);
  }

  output_file = fopen(CONF_tty_path, "r+");
  if (output_file < 0) {
    fprintf(stderr, "error %d opening %s: %s", errno, CONF_tty_path,
            strerror(errno));
    abort();
  }

  set_interface_attribs(output_file, B230400, 0);
  set_blocking(output_file, 1);

  //
  // Send HELO
  //
  send_message(output_file, 1, (uint8_t *)"HELO", 4);

  //
  // Send everything
  //
  VTermRect rect = {0};
  rect.start_row = 0;
  rect.end_row = pt->rows;
  rect.start_col = 0;
  rect.end_col = pt->cols;

  send_rectangle(output_file, &rect, pt);
}

void ledian_pangoterm_free(PangoTerm *pt) {
  if (ScreenBuffer != NULL) {
    free(ScreenBuffer);
    ScreenBuffer = NULL;
  }

  if (output_file != NULL) {
    fclose(output_file);
    output_file = NULL;
  }
}
