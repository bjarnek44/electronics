#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include "nmea_0183_utils.h"

FILE*  open_tty_file( char*  name,
		      int    baud,
                      int    is_output )
{
  FILE*           fp;
  int             fd;
  int             flags  =  O_NOCTTY | O_NDELAY;
  struct termios  options;

  if (is_output) {
    flags  |=  O_WRONLY;
  }
  else {
    flags  |=  O_RDONLY;
  }

  if (name == NULL) {
    return  stdin;
  }

  fd  =  open(name, flags);

  if (fd < 0) {
    fprintf(stderr, "Error opening input file %s\n", name);
    exit(1);
  }

  if (fcntl(fd, F_SETFL, 0) != 0) {
    fprintf(stderr, "Error setting input file flags.\n");
    exit(1);
  }

  if (tcgetattr(fd, &options) != 0) {
    fprintf(stderr, "Error getting tty file attributes.\n");
    exit(1);
  }

  cfmakeraw(&options);

  if (baud == 115200) {
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
  }
  else if (baud == 38400) {
    cfsetispeed(&options, B38400);
    cfsetospeed(&options, B38400);
  }
  else if (baud == 4800) {
    cfsetispeed(&options, B4800);
    cfsetospeed(&options, B4800);
  }
  else {
    fprintf(stderr, "Unknown baud rate: %d\n", baud);
    exit(1);
  }

  options.c_cflag  |=  CLOCAL;

  if (!is_output) {
    options.c_cflag  |=  CREAD;
  }

  if (tcsetattr(fd, TCSANOW, &options) != 0) {
    fprintf(stderr, "Error setting tty file attributes.\n");
    exit(1);
  }

  if (is_output) {
    fp  =  fdopen(fd, "w");
  }
  else {
    fp  =  fdopen(fd, "r");
  }

  if (fp == NULL) {
    fprintf(stderr, "Error opening tty file.\n");
    exit(1);
  }

  return  fp;
}


void  close_tty_file( FILE*  fp )
{
  if (fp != stdin && fclose(fp) != 0) {
    fprintf(stderr, "Problem closing input file.\n");
    exit(1);
  }
}
