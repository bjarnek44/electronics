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

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#define MAX_LINE         1024

#define FIFO_IDX_NO      -1
#define FIFO_IDX_STDOUT  -2
#define FIFO_CNT          8   // Maximum number of fifos

typedef struct {
  FILE*  fp;

  char*  name;
} fifo_t;


void  usage() {
  fprintf(stderr, "\n");
  fprintf(stderr, "usage: nmea_split [options] -f <channels> <fifo file>\n");
  fprintf(stderr, "                           [-f <channels> <fifo file>] ..\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "Takes input from stdin (typically the output of nmea_0183_read) and splits it\n");
  fprintf(stderr, "into different newly created fifo files and/or stdout according to the NMEA\n");
  fprintf(stderr, "0183 channel it came from. Make sure the NMEA outputs includes the channel\n");
  fprintf(stderr, "number as the first character of each sentence.\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "Options:\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "  -h: print this help.\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "  -f <channels> <fifo file>: \"channels\" is any number of digits from 1 to 8\n");
  fprintf(stderr, "        indicating input channels to be put in a fifo file. \"fifo file\" is a\n");
  fprintf(stderr, "        file name for a new fifo to be created. \"-\" indicates stdout. This\n");
  fprintf(stderr, "        option can be used several times.\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "Example:\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "  nmea_split -f 123 /tmp/nmea -f 456 - -f 7 /tmp/navtex\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "Here, channels 1-3 go to a fifo called /tmp/nmea while channel 4-6 goes to\n");
  fprintf(stderr, "stdout. Channel 7 goes to a fifo called /tmp/navtex and channel 8 is\n");
  fprintf(stderr, "ignored. The fifos are created by this program. The input is read from stdin.\n");
  fprintf(stderr, "\n");

  exit(1);
}


int  main( int     argc,
           char**  argv )
{
  FILE*       in_fp       =  stdin;
  int         fifo_found  =  0;
  int         p           =  1;
  int         fifo_count  =  0;
  fifo_t      fifos[FIFO_CNT];
  int         fifo_indices[FIFO_CNT];
  char        s[MAX_LINE];
  int         i;

  for (i = 0; i < FIFO_CNT; i++) {
    fifo_indices[i]  =  FIFO_IDX_NO;
  }

  while (p < argc) {
    if (strcmp(argv[p], "-h") == 0) {
      usage();
    }
    else if (strcmp(argv[p], "-f") == 0) {
      int  idx  =  FIFO_IDX_STDOUT;

      if (p + 1 >= argc) {
        fprintf(stderr, "No fifo channels given.\n");
        usage();
      }

      if (p + 2 >= argc) {
        fprintf(stderr, "No fifo file given.\n");
        usage();
      }

      if (strcmp(argv[p + 2], "-") != 0) {
        idx = fifo_count;
      }
      else {
        for (i = 0; i < FIFO_CNT; i++) {
          if (fifo_indices[i] == FIFO_IDX_STDOUT) {
            fprintf(stderr, "stdout given as output twice.\n");
            usage();
          }
        }
      }

      for (i = 0; argv[p + 1][i] != '\0'; i++) {
        if (argv[p + 1][i] < '1' || argv[p + 1][i] > '0' + FIFO_CNT) {
          fprintf(stderr, "Wrong channel number: %c\n", argv[p + 1][i]);
          usage();
        }

        if (fifo_indices[argv[p + 1][i] - '1'] != FIFO_IDX_NO) {
          fprintf(stderr, "Fifo for channel %c given twice.\n", argv[p + 1][i]);
          usage();
        }
        fifo_indices[argv[p + 1][i] - '1'] = idx;
      }

      if (idx != FIFO_IDX_STDOUT) {
        for (i = 0; i < fifo_count; i++) {
          if (strcmp(fifos[i].name, argv[p + 2]) == 0) {
            fprintf(stderr, "Fifo name %s given twice.\n", argv[p + 2]);
            usage();
          }
        }

        fifos[fifo_count].name  =  argv[p + 2];

        fifo_count++;
      }

      p  +=  3;

      fifo_found  =  1;
    }
    else {
      fprintf(stderr, "Unknown option: %s\n", argv[p]);
      usage();
    }
  }

  if (!fifo_found) {
    fprintf(stderr, "No -f option found.\n");
    usage();
  }


  for (i = 0; i < fifo_count; i++) {
    char*  name  =  fifos[i].name;

    if (name == NULL) {
      // stdout
      continue;
    }

    struct stat  stat_str;

    if (stat(name, &stat_str) != 0) {
      if (errno != ENOENT) {
        fprintf(stderr, "Error checking fifo file: %s\n", name);
        exit(1);
      }
    }
    else if ((stat_str.st_mode & S_IFIFO) != 0) {
      // Remove fifo if it already exists
      if (unlink(name) != 0) {
        fprintf(stderr, "Error removing existing fifo file: %s\n", name);
        exit(1);
      }
    }

    if (mkfifo(name, 0666) != 0) {
      fprintf(stderr, "Error creating fifo file: %s\n", name);
      exit(1);
    }
  }

  for (i = 0; i < fifo_count; i++) {
    char*  name  =  fifos[i].name;
    int    fd;

    if (name == NULL) {
      // stdout
      continue;
    }

    fd  =  open(name, O_WRONLY);

    if (fd < 0) {
      fprintf(stderr, "Error opening fifo: %s\n", name);
      unlink(name);
      exit(1);
    }

    fifos[i].fp  =  fdopen(fd, "w");

    if (fifos[i].fp == NULL) {
      fprintf(stderr, "Error opening fifo: %s\n", name);
      unlink(name);
      exit(1);
    }
  }

  while (fgets(s, MAX_LINE, in_fp) != NULL) {
    if (s[0] < '1' || s[0] > '0' + FIFO_CNT) {
      fprintf(stderr, "Wrong channel number in input: %s", s);
    }
    else {
      int  idx  =  fifo_indices[s[0] - '1'];

      if (idx == FIFO_IDX_STDOUT) {
        fputs(s + 1, stdout);
        fflush(stdout);
      }
      else if (idx != FIFO_IDX_NO) {
        fputs(s + 1, fifos[idx].fp);
        fflush(fifos[idx].fp);
      }
    }
  }

  for (i = 0; i < fifo_count; i++) {
    char*  name  =  fifos[i].name;

    if (name == NULL) {
      // stdout
      continue;
    }

    if (fclose(fifos[i].fp) != 0 || unlink(name) != 0) {
      fprintf(stderr, "Error closing fifo: %s\n", fifos[i].name);
    }
  }

  return  0;
}
