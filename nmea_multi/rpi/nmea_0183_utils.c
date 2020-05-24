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

  if (name == NULL) {
    if (is_output) {
      return  stdout;
    }
    else {
      return  stdin;
    }
  }

  if (is_output) {
    flags  |=  O_WRONLY;
  }
  else {
    flags  |=  O_RDONLY;
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
  if (fp != stdin && fp != stdout && fclose(fp) != 0) {
    fprintf(stderr, "Problem closing input file.\n");
    exit(1);
  }
}
