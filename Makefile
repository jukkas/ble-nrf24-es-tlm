DEVICE=stm8s103f3

## A directory for common include files
#COMMONDIR = ..

## Get program name from enclosing directory name
PROGRAM = $(lastword $(subst /, ,$(CURDIR)))

SOURCES=$(wildcard *.c)
OBJECTS=$(SOURCES:.c=.rel)
HEADERS=$(wildcard *.h)

CC = sdcc
PROGRAMMER = stlinkv2


DEFINES=
DEFINES += -DSTM8S103

## If you want beacon to use "Health thermometer" service (default is "Temperature")
#DEFINES += -DNORDIC_TEMPERATURE

CPPFLAGS = -I.
CFLAGS = --Werror --std-sdcc99 -mstm8 $(DEFINES)
LDFLAGS = -lstm8 -mstm8 --out-fmt-ihx

.PHONY: all clean flash

$(PROGRAM).ihx: $(OBJECTS)
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

%.rel : %.c $(HEADERS)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c -o $@ $<

CCOMPILEDFILES=$(SOURCES:.c=.asm) $(SOURCES:.c=.lst) $(SOURCES:.c=.rel) \
               $(SOURCES:.c=.rst) $(SOURCES:.c=.sym)
clean:
	rm -f $(PROGRAM).ihx $(PROGRAM).cdb $(PROGRAM).lk $(PROGRAM).map $(CCOMPILEDFILES)

flash: $(PROGRAM).ihx
	stm8flash -c $(PROGRAMMER) -p $(DEVICE) -w $(PROGRAM).ihx
