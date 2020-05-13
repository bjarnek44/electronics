ifeq ($(TOOLS),)
$(error Please set the TOOLS variable to the gputils folder)
endif

PROC	:= 16f1705
ASM	:= $(TOOLS)/bin/gpasm
LINK	:= $(TOOLS)/bin/gplink
INCLUDE	:= $(TOOLS)/share/gputils/header
SCRIPT	:= $(TOOLS)/share/gputils/lkr/$(PROC)_g.lkr

all: nmea_multi.hex

%.o: %.asm
	$(ASM) -c -p $(PROC) $<

nmea_multi.o: nmea_multi.asm
	$(ASM) -o $@ -c -p $(PROC) -I $(INCLUDE) $<

%.hex: %.o
	$(LINK) -s $(SCRIPT) $< -o $@

clean:
	rm -f *.o *.cod *.lst *~ MPLABXLog.xml*

proper: clean
	rm -f *.hex
