#define __GLIBC_USE
#include "freertos/FreeRTOS.h"

#include "Common.h"
#include "Command.h"

void command_parser_main(void *) {
  while (true) {
    if (Command::parse()) {
    }
  }
}