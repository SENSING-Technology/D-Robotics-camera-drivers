CUR_DIR := $(shell pwd)
TARGET = matrix_multiply

SRC_PATH += $(CURDIR)

SRCS := $(foreach cf, $(SRC_PATH), $(wildcard $(cf)/*.c))
OBJS :=  $(SRCS:.c=.o)

LDFLAGS := -L/usr/hobot/lib
LDFLAGS += -lOpenCL -lm -lEGL -lGLESv2
INCDIR := -I/usr/hobot/include
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
