#ifndef __nmea_0183_utils_h__
#define __nmea_0183_utils_h__

#include <stdio.h>

#define CONFIG_GPIO 3  // default GPIO for configuration

FILE*  open_tty_file( char*  name,
		      int    baud,
                      int    is_output );

void  close_tty_file( FILE*  fp );

#endif // __nmea_0183_utils_h__
