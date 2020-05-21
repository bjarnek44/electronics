CFLAGS := -Wall -Werror -O3
LDFLAGS := -lpthread -lm

TARGETS := nmea_0183_read nmea_0183_config nmea_split
# TODO: add later: topline_to_nmea nmea_2000_to_0183

all: $(TARGETS)

%.o: %.c
	gcc $(CFLAGS) -c $<

nmea_0183_read: nmea_0183_read.o nmea_0183_utils.o
	gcc $(LDFLAGS) -o $@ $^ -lgpiod

nmea_0183_config: nmea_0183_config.o nmea_0183_utils.o
	gcc $(LDFLAGS) -o $@ $^ -lgpiod -lpthread

nmea_split: nmea_split.o
	gcc $(LDFLAGS) -o $@ $^

nmea_2000_to_0183: nmea_2000_to_0183.o nmea_2000_coll.o nmea_2000_gps_conv.o nmea_2000_ais_conv.o nmea_2000_misc_conv.o nmea_2000_conv.o nmea_2000_utils.o
	gcc $(LDFLAGS) -o $@ $^ -lgpiod

topline_to_nmea: topline_to_nmea.o
	gcc $(LDFLAGS) -o $@ $^

clean:
	rm -f *~ *.o

proper: clean
	rm -f $(TARGETS)
