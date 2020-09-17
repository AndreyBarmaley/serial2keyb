# makefile
#
SOURCEROOT	:= ../src
SRCDIRLIST	:= $(SOURCEROOT)
SEARCH		:= $(addsuffix /*.c, $(SRCDIRLIST))

$(TARGET): $(notdir $(patsubst %.c, %.o, $(wildcard $(SEARCH))))
	$(CXX) $^ $(LIBS) -o $@ $(LDFLAGS)

VPATH		:= $(SRCDIRLIST)

%.o: %.c
	$(CXX) -c -MD $< $(CFLAGS) $(addprefix -I, $(SRCDIRLIST))

include $(wildcard *.d)

.PHONY: clean

clean:
	rm -f *.d *.o $(TARGET)
