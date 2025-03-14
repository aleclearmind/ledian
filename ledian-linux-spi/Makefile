CC	= gcc
LD	= gcc
CFLAGS	= -O1 -Wall -Wextra -Wno-unused-parameter -MMD
LDFLAGS	=

SOURCES	= $(wildcard *.c)
OBJECTS	= $(patsubst %.c,%.o,$(SOURCES))
DEPENDS	= ${OBJECTS:.o=.d}

TARGET= ledian

.DEFAULT_GOAL: all

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

$(TARGET): $(OBJECTS)
	$(LD) $(LDFLAGS) -o $@ $(OBJECTS)

.PHONY: all
all: $(TARGET)

.PHONY: clean
clean:
	rm -f $(OBJECTS)
	rm -f $(DEPENDS)
	rm -f $(TARGET)

-include ${DEPENDS}
