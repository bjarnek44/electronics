// Copyright 2020 Bjarne Knudsen
//
// Redistribution and use in source and binary forms, with or without modification, are permitted
// provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this list of
// conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice, this list of
// conditions and the following disclaimer in the documentation and/or other materials provided
// with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors may be used to
// endorse or promote products derived from this software without specific prior written
// permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
// OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <gpiod.h>

#include "nmea_0183_utils.h"

#define MAX_LINE         1024

void  usage() {
  fprintf(stderr, "\n");
  fprintf(stderr, "usage: nmea_0183_read [options]\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "Reads from a tty input device and prints to stdout. By default, the reading\n");
  fprintf(stderr, "stops when GPIO %d is pulled down for configuration of NMEA 0183 multiplexer.\n", CONFIG_GPIO);
  fprintf(stderr, "\n");
  fprintf(stderr, "  -h: print this help.\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "  -b <rate>: one of these: 4800, 38400, 115200. Default is 115000.\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "  -i <device>: a tty input device or \"-\" for stdin. /dev/ttyAMA0 is default.\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "  -g <pin>: GPIO pin for config mode, \"-\" for no pin. %d is default.\n", CONFIG_GPIO);
  fprintf(stderr, "\n");

  exit(1);
}


int  main( int     argc,
           char**  argv )
{
  struct gpiod_chip*  chip        =  NULL;
  struct gpiod_line*  line        =  NULL;
  FILE*               in_fp;
  char*               input_name  =  NULL;
  int                 baud        =  -1;
  int                 gpio        =  -2; // -2 is not set, -1 is no gpio
  int                 p           =  1;
  int                 i;
  char                s[MAX_LINE];

  while (p < argc) {
    if (strcmp(argv[p], "-h") == 0) {
      usage();
    }
    else if (strcmp(argv[p], "-b") == 0) {
      if (baud != -1) {
        fprintf(stderr, "Baud rate given twice\n");
        usage();
      }

      if (p + 1 >= argc) {
        fprintf(stderr, "No baud rate given\n");
        usage();
      }

      if (sscanf(argv[p + 1], "%d%n", &(baud), &i) < 1 || argv[p + 1][i] != '\0') {
        fprintf(stderr, "Wrong baud rate\n");
        usage();
      }

      if (baud != 4800 && baud != 38400 && baud != 115200) {
        fprintf(stderr, "Wrong baud rate\n");
        usage();
      }

      p  +=  2;
    }
    else if (strcmp(argv[p], "-i") == 0) {
      if (input_name != NULL) {
        fprintf(stderr, "Input device given twice\n");
        usage();
      }

      if (p + 1 >= argc) {
        fprintf(stderr, "No input device given\n");
        usage();
      }

      input_name = argv[p + 1];
      p  +=  2;
    }
    else if (strcmp(argv[p], "-g") == 0) {
      if (gpio != -2) {
        fprintf(stderr, "GPIO given twice\n");
        usage();
      }

      if (p + 1 >= argc) {
        fprintf(stderr, "No GPIO given\n");
        usage();
      }

      if (strcmp(argv[p + 1], "-") == 0) {
        gpio  =  -1;
      }
      else if (sscanf(argv[p + 1], "%d%n", &gpio, &i) < 1 || argv[p + 1][i] != '\0' || gpio < 0) {
        fprintf(stderr, "Wrong GPIO\n");
        usage();
      }

      p  +=  2;
    }
    else {
      fprintf(stderr, "Unknown option: %s\n", argv[p]);
      usage();
    }
  }

  if (baud == -1) {
    baud  =  115200;
  }

  if (input_name == NULL) {
    input_name  =  "/dev/ttyAMA0";
  }
  else if (strcmp(input_name, "-") == 0) {
    // use NULL to indicate stdin
    input_name  =  NULL;
  }

  if (gpio == -2) {
    gpio  =  CONFIG_GPIO;
  }

  if (gpio != -1) {
    chip  =  gpiod_chip_open("/dev/gpiochip0");

    if (chip == NULL) {
      fprintf(stderr, "Error opening GPIO chip\n");
      exit(1);
    }
  }

  in_fp  =  open_tty_file(input_name, baud, 0);

  while (fgets(s, MAX_LINE, in_fp) != NULL) {
    // we need this to update used status
    line  =  gpiod_chip_get_line(chip, gpio);

    if (line == NULL) {
      fprintf(stderr, "Error opening GPIO line\n");
      exit(1);
    }

    if (gpio != -1 && gpiod_line_is_used(line)) {
      // configuration, discard string read

      fprintf(stderr, "Entering configuration mode\n");

      close_tty_file(in_fp);

      while (gpiod_line_is_used(line)) {
        sleep(1);

        line  =  gpiod_chip_get_line(chip, gpio);

        if (line == NULL) {
          fprintf(stderr, "Error opening GPIO line\n");
          exit(1);
        }
      }

      fprintf(stderr, "Exiting configuration mode\n");

      in_fp  =  open_tty_file(input_name, baud, 0);
    }
    else {
      fputs(s, stdout);
      fflush(stdout);
    }
  }

  if (chip != NULL) {
    gpiod_chip_close(chip);
  }

  return  0;
}
