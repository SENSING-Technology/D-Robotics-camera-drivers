TARGET = sample_2v_219_stitch_codec

SRC_PATH := $(CURDIR)
HOBOT_ROOTFS_DIR := /usr/hobot

SRCS := $(foreach cf, $(SRC_PATH), $(wildcard $(cf)/*.c))
OBJS :=  $(SRCS:.c=.o)

LIBS := -lpthread -ldl -lhbmem -lalog -lmultimedia
LIBS += -lavformat -lavcodec -lavutil -lswresample -lcam -lvpf

INCS := -I${HOBOT_ROOTFS_DIR}/include
LDFLAGS := -L${HOBOT_ROOTFS_DIR}/lib

.PHONY: all clean

all: ${TARGET}

%.o:%.c
	@mkdir -p $(abspath $(dir $@))
	$(CC) -o $@ $(INCS) -c $<

$(TARGET):$(OBJS)
	@mkdir -p $(abspath $(dir $@))
	$(CC) -o $@ $(OBJS) ${INCS} $(LDFLAGS) $(LIBS)

clean:
	rm -rf ${OBJS} ${TARGET}
	rm -rf *h264
