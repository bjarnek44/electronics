#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <gpiod.h>

#include "nmea_0183_utils.h"

#define MAX_LINE         1024


typedef struct {
  FILE*  in_fp;

  int    is_ready;
} thr_arg_t;


void  usage() {
  fprintf(stderr, "\n");
  fprintf(stderr, "usage: nmea_0183_config [options]\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "Configures a NMEA 0183 multiplexer via a tty device. A GPIO pin is pulled down\n");
  fprintf(stderr, "to make the multiplexer enter configuration mode and then communication\n");
  fprintf(stderr, "starts. The \"X\" command exits the configuration program and releases the GPIO\n");
  fprintf(stderr, "pin to let the multiplexer exit configuation mode.\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "  -h: print this help.\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "  -b <rate>: one of these: 4800, 38400, 115200. Default is 4800 and almost\n");
  fprintf(stderr, "        always right.\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "  -i <device>: a tty input device. /dev/ttyAMA0 is default.\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "  -g <pin>: GPIO pin for config mode, \"-\" for no pin. %d is default.\n", CONFIG_GPIO);
  fprintf(stderr, "\n");

  exit(1);
}


void*  copy_input( void*  void_arg )
{
  thr_arg_t*  arg  =  (thr_arg_t*) void_arg;
  char        s[MAX_LINE];

  while (fgets(s, MAX_LINE, arg->in_fp) != NULL) {
    if (arg->is_ready) {
      printf("  ");
      fputs(s, stdout);
    }
  }

  return  NULL;
}


int  main( int     argc,
	   char**  argv )
{
  struct gpiod_chip*  chip        =  NULL;
  struct gpiod_line*  line        =  NULL;
  int                 gpio        =  -2; // -2 is not set, -1 is no gpio
  char*               input_name  =  "/dev/ttyAMA0";
  int                 baud        =  -1;
  int                 p           =  1;
  char                s[MAX_LINE];
  FILE*               out_fp;
  pthread_t*          thread;
  thr_arg_t*          arg;


  while (p < argc) {
    int  n;

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

      if (sscanf(argv[p + 1], "%d%n", &(baud), &n) < 1 || argv[p + 1][n] != '\0') {
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
      else if (sscanf(argv[p + 1], "%d%n", &gpio, &n) < 1 || argv[p + 1][n] != '\0' || gpio < 0) {
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
    baud  =  4800;
  }

  if (input_name == NULL) {
    input_name  =  "/dev/ttyAMA0";
  }

  if (gpio == -2) {
    gpio  =  CONFIG_GPIO;
  }

  chip = gpiod_chip_open("/dev/gpiochip0");

  if (chip == NULL) {
    fprintf(stderr, "Error opening GPIO chip\n");
    exit(1);
  }

  line  =  gpiod_chip_get_line(chip, gpio);

  if (line == NULL) {
    fprintf(stderr, "Error opening GPIO line\n");
    exit(1);
  }

  if (gpiod_line_request_output(line, "nmea_0183_config", 0) != 0) {
    fprintf(stderr, "Error requesting GPIO output\n");
    exit(1);
  }

  arg     =  (thr_arg_t*) malloc(sizeof(thr_arg_t));
  thread  =  (pthread_t*) malloc(sizeof(pthread_t));

  printf("[starting...]\n");

  arg->in_fp     =  open_tty_file(input_name, baud, 0);
  arg->is_ready  =  0;

  pthread_create(thread, NULL, copy_input, arg);

  sleep(1);

  printf("[ready]\n");

  arg->is_ready  =  1;
  out_fp         =  open_tty_file(input_name, baud, 1);

  while (fgets(s, MAX_LINE, stdin) != NULL) {
    if (strcmp(s, "X\n") == 0) {
      break;
    }

    fputs(s, out_fp);
  }

  printf("[done]\n");

  // it would be nicer if we could join the thread, but too much
  // trouble since fgets() in work thread is blocking.
  pthread_cancel(*thread);

  close_tty_file(arg->in_fp);
  close_tty_file(out_fp);

  free(thread);
  free(arg);

  // make the GPIO an input and don't worry if it fails
  gpiod_line_release(line);
  gpiod_line_request_input(line, "nmea_0183_config");

  gpiod_chip_close(chip);

  return  0;
}
