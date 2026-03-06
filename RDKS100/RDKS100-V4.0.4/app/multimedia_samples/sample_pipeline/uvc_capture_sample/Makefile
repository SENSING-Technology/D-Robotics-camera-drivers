include ../../Makefile.in
TARGET = uvc_capture_sample

SRC_PATH += $(CURDIR)

SRCS := $(wildcard *.c)
OBJS := $(SRCS:.c=.o)

LDFLAGS := -L/usr/hobot/lib
LDFLAGS += -lvpf -lvio -lcam -lhbmem
INCDIR := -I/usr/hobot/include -I/usr/include/cjson/
INCDIR += $(INCS)

.PHONY: all clean

all: ${TARGET}

%.o:%.c
		@mkdir -p $(abspath $(dir $@))
		$(CC) -o $@ $(INCDIR) -c $<

$(TARGET):$(OBJS)
		@mkdir -p $(abspath $(dir $@))
		$(CC) -O3 -o $@ $(OBJS) $(LDFLAGS)

clean:
	rm -rf $(TARGET) *.o
