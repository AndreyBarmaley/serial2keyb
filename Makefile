# makefile
#
TARGET		:= serial2keyb
MAKE		:= make

CXX		:= gcc
#CFLAGS          := -O0 -g -Wall -Wno-stringop-truncation
CFLAGS          := -O2 -Wall -Wno-stringop-truncation
LDFLAGS         :=
LIBS            := 

export		CXX LDFLAGS CFLAGS LIBS TARGET

all:
	$(MAKE) -C dist
	@cp ./dist/$(TARGET) .

clean:
	$(MAKE) -C dist clean
	rm -f $(TARGET)
