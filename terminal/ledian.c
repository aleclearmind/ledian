#define _GNU_SOURCE
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <math.h>
#include <sys/random.h>

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

#if 1
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
  tty.c_cflag &= ~CRTSCTS;
#endif

  // tty.c_iflag |= IXON | IXOFF;
  // tty.c_cflag |= CRTSCTS;

  tty.c_iflag &= ~ONLCR;
  tty.c_iflag &= ~ICRNL;

  // cfmakeraw(&tty);

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    fprintf(stderr, "error %d from tcsetattr\n", errno);
  }

  return 0;
}

static void set_blocking(FILE *file, int should_block) {
  return;
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

typedef struct {
  uint8_t *buffer;
  uint8_t *match;
}  ReadUntilResult;

static ReadUntilResult read_until(FILE *file, char *what, size_t size) {
  ReadUntilResult result;

  uint8_t *Buffer = calloc(1, 1024 * 1024 * 2);
  uint8_t *BufferPointer = Buffer;
  uint8_t *match = NULL;
  int fd = fileno(file);
  struct pollfd Query = {fd, POLLIN, 0};
  while (match == NULL || poll(&Query, 1, 0) == 1) {
    size_t Length = read(fd, BufferPointer, 1024);
    if (Length == -1)
      continue;
    fwrite(BufferPointer, 1, Length, stderr);
    fflush(stderr);
    BufferPointer += Length;
    match = memmem(&Buffer[0], BufferPointer - &Buffer[0], what, size);

    if (match != NULL && memmem(match, BufferPointer - match, "\n", 1) == NULL)
      match = NULL;
  }

  result.buffer = Buffer;
  result.match = match;
  return result;
}

static void wait_for_ack(FILE *file) {
  return;
  fprintf(stderr, "Wait for ACK\n");
  ReadUntilResult read_result = read_until(file, "ACK", 3);
  free(read_result.buffer);
  fprintf(stderr, "Got ACK\n");
}

static void strict_write(FILE *file, const void *buf, size_t count) {
  ssize_t Result = fwrite(buf, 1, count, file);
  assert(Result == count);
}

typedef uint8_t ledian_id_t;
typedef uint32_t length_t;

static void send_message(FILE *file, ledian_id_t ID, uint8_t *Data,
                         length_t Size) {
  assert(sizeof(Size) == 4);
  fprintf(stderr, "Sending message with ID %d, size %d:\n  ", ID, Size);

  for (size_t I = 0; I < Size; ++I)
    fprintf(stderr, "%.2x ", Data[I]);
  fprintf(stderr, "\n");
  

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

typedef struct RGBColor {
	int r;
	int g;
	int b;
} RGBColor;

static RGBColor hsv2rgb(float H, float S, float V) {
	float r = 0.;
	float g = 0.;
	float b = 0.;
	
	float h = H / 360;
	float s = S / 100;
	float v = V / 100;
	
	int i = floor(h * 6);
	float f = h * 6 - i;
	float p = v * (1 - s);
	float q = v * (1 - f * s);
	float t = v * (1 - (1 - f) * s);
	
	switch (i % 6) {
		case 0: r = v, g = t, b = p; break;
		case 1: r = q, g = v, b = p; break;
		case 2: r = p, g = v, b = t; break;
		case 3: r = p, g = q, b = v; break;
		case 4: r = t, g = p, b = v; break;
		case 5: r = v, g = p, b = q; break;
	}
	
	RGBColor color;
	color.r = r * 255;
	color.g = g * 255;
	color.b = b * 255;
	
	return color;
}

typedef struct {
  uint32_t Column;
  uint32_t Line;
} cursor_t;

/// Send a specific cell to file
static void send_cell(FILE *file, PangoTerm *pt, int Line, int Column) {
  uint8_t Value = *(ScreenBuffer + Line * pt->cols + Column);
  bool IsCursor = Line == ScreenCursor.row && Column == ScreenCursor.col;

  assert(sizeof(ColorMap) == 256 * 3);

  HSV Color = ColorMap[Value];
  RGBColor RGB = hsv2rgb(Color.H, Color.S, Color.V);

  uint8_t Buffer[4] = {RGB.r, RGB.g, RGB.b, IsCursor ? 0x01 : 0x00};
  fprintf(stderr, "  %.2x %.2x %.2x %.2x\n", Buffer[0], Buffer[1], Buffer[2], Buffer[3]);
  strict_write(file, &Buffer, 4);
  fflush(file);
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

  g_assert(sizeof(cursor_t) == 8);

  for (size_t Line = rect->start_row; Line < rect->end_row; ++Line) {
    // Move cursor to new line
    cursor_t Cursor = {rect->start_col, Line};
    send_message(file, 3, (uint8_t *)&Cursor, sizeof(Cursor));

    fprintf(stderr, "Sending UpdateRange\n");
    unsigned Start = rect->start_col;
    static unsigned MaxSend = 8192;
    while (Start < rect->end_col) {
      unsigned End = MIN(rect->end_col, Start + MaxSend);

      length_t Length = 4 * (End - Start);
      strict_write(file, "\x02", 1);
      strict_write(file, &Length, 4);
      for (size_t Column = Start; Column < End; ++Column) {
        send_cell(file, pt, Line, Column);
      }
      wait_for_ack(file);

      Start += MaxSend;
    }
  }
}

void ledian_term_movecursor(PangoTerm *pt, VTermPos pos) {
  return;

  ScreenCursor = pos;

  VTermRect CursorRect = {
    .start_row = pos.row,
    .end_row = pos.row + 1,
    .start_col = pos.col,
    .end_col = pos.col + 1
  };
  ledian_term_damage(pt, CursorRect);
}

void ledian_term_damage(PangoTerm *pt, VTermRect rect) {
  fprintf(stderr, "term_damage({ %d, %d, %d, %d });\n", rect.start_row,
          rect.end_row, rect.start_col, rect.end_col);

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
  fprintf(stderr, "ledian_pangoterm_start\n");

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

  set_interface_attribs(output_file, /* B1000000 */ B460800 /* B115200 */, 0);
  set_blocking(output_file, 1);

  //
  // Send HELO
  //
  send_message(output_file, 1, (uint8_t *)"HELO", 4);

  // Disable debug messages
  send_message(output_file, 4, (uint8_t *)"\x00", 1);

  ledian_selftest();

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

void ledian_stream(void) {
  while (true) {
    cursor_t Cursor = { 0, 0 };
    send_message(output_file, 3, (uint8_t *)&Cursor, sizeof(Cursor));

#define LINES 24
#define COLUMNS 80
    static char buffer[3 * COLUMNS * LINES];
    static char buffer2[4 * COLUMNS * LINES];

    size_t ReadBytes = 0;
    while (ReadBytes != 3 * COLUMNS * LINES) {
      ReadBytes += fread(&buffer[ReadBytes], 1, 3 * COLUMNS * LINES - ReadBytes, stdin);
    }

    for (unsigned I = 0; I < LINES; ++I) {
      // assert(ReadBytes == 3 * COLUMNS);
      for (unsigned J = 0; J < COLUMNS; ++J) {
        size_t Index = (I * COLUMNS + J);
        buffer2[Index * 4 + 0] = buffer[Index * 3 + 0];
        buffer2[Index * 4 + 1] = buffer[Index * 3 + 1];
        buffer2[Index * 4 + 2] = buffer[Index * 3 + 2];
      }
    }

    send_message(output_file, 2, &buffer2, LINES * COLUMNS * 4);
  }
}

void fill_buffer_with_random(char *buffer, size_t size) {
    if (buffer == NULL || size == 0) {
        fprintf(stderr, "Invalid buffer or size.\n");
        return;
    }

    ssize_t result = getrandom(buffer, size, 0);
    if (result == -1) {
        perror("getrandom failed");
    } else if ((size_t)result < size) {
        fprintf(stderr, "getrandom returned fewer bytes than requested.\n");
    }
}

uint32_t crc32b(unsigned char *Message, size_t Size) {
   uint32_t Result = 0xFFFFFFFF;

   for (size_t I = 0; I < Size; ++I) {
      Result = Result ^ Message[I];
      for (int J = 7; J >= 0; J--) {
        Result = (Result >> 1) ^ (0xEDB88320 & (-(Result & 1)));
      }
   }

   return ~Result;
}

void ledian_selftest(void) {
  for (unsigned I = 4; I < 8192 + 1; I *= 2) {
    for (unsigned J = 0; J < 1; ++J) {
      char *buffer = calloc(I, 1);
      fill_buffer_with_random(buffer, I);
      uint32_t expected_crc = crc32b(buffer, I);
      fprintf(stderr, "Size: %d, Expected CRC: %" PRIu32 "\n", I, expected_crc);
      send_message(output_file, 5, buffer, I);
      free(buffer);

      char *prefix = "CRC32: ";
      size_t prefix_length = strlen(prefix);
      ReadUntilResult read_result = read_until(output_file, prefix, prefix_length);
      uint32_t actual_crc = atoll(read_result.match + prefix_length);
      free(read_result.buffer);
      assert(actual_crc == expected_crc);
    }
  }
}
