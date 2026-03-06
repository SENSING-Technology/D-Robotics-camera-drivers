include ../Makefile.in
CUR_DIR := $(shell pwd)

TARGET = sample_codec
SRC_PATH += $(CURDIR)

SRCS := $(foreach cf, $(SRC_PATH), $(wildcard $(cf)/*.c))
OBJS :=  $(SRCS:.c=.o)

LDFLAGS := -L/usr/hobot/lib
LDFLAGS += -lvpf -lvio -lcam -lhbmem -lmultimedia -lavformat -lavcodec -lavutil -lswresample
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
	rm -rf ${OBJS} ${TARGET}
