CUR_DIR := $(shell pwd)
TARGET = bezier

SRC_PATH += $(CURDIR)
SRC_PATH += ${CUR_DIR}/../common
SRCS := $(foreach cf, $(SRC_PATH), $(wildcard $(cf)/*.c))
OBJS :=  $(SRCS:.c=.o)

INCS += -I $(CUR_DIR)/../common/

LDFLAGS := -L/usr/hobot/lib
LDFLAGS += -lOpenCL -lm -lEGL -lGLESv2 -lwayland-client -lwayland-egl
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
