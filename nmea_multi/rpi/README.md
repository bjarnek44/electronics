Overview
========

The programs in this folder are used to communicate with the NMEA
multiplexer. The NMEA 0183 related programs are:

  * nmea_0183_read: Reads from the multiplexer
  * nmea_split: Splits the input into different fifos
  * nmea_0183_config: configures the multiplexer

This is a typical use of the two programs for data input:

```
nmea_0183_read | nmea_split -f 234 /tmp/nmea.fifo -f 1 /tmp/navtex.fifo
```

The nmea_0183_config program is made to configure the multiplexer and
tells nmea_0183_read to stop reading while the configuration is going
on. This way, the reader program can be kept running and no
configuration information is forwarded to devices using the NMEA 0183
data.

nmea_0183_read by itself just outputs data from the multiplexer to
stdout, so it can be used by itself to see the NMEA 0183 data.

Each program can be run with the -h option to get information about
how to use it.


Requirements
============

Using the Raspbian operating system, the following is needed for the
progams to run:

```
sudo adduser <user name> gpio
sudo apt-get install libgpiod-dev gpiod screen
```

The first line adds your user to the gpio group which allows access to
the GPIO pins as a regular user.

The gpiod package is not strictly needed, but has some good command
line tools for handling the GPIOs on the Raspberry Pi. The screen
package is also not strictly needed, but the screen program is very
useful for running the reading program in the background.

I use [kplex](http://www.stripydog.com/kplex/) for forwarding the NMEA
0183 data on my network.


Building
========

After installing the required packages, you should be able to just
run:

```
make
```
