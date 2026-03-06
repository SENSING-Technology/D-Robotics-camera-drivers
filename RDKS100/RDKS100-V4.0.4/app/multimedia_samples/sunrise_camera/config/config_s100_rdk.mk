
GLOBAL_INSTALL_DIR := $(PRO_ROOT)sunrise_camera
CFLAGS_EX  := -Wall -g -O2 -fstack-protector -Wno-error=unused-result  -Wno-unused-result

HBRE_LIB ?= /usr/hobot/lib
HBRE_INC ?= /usr/hobot

CHIP_ID ?= CHIP_S100_RDK
############################################################
# MODULE_SYSTEM := y
MODULE_VPP := y
# MODULE_NETWORK := y
# MODULE_RECORD := y
# MODULE_ALARM := y
# MODULE_RTSP := y
MODULE_WEBSOCKET := y
MODULE_MEDIA_SERVER := y
# MODULE_ENABLE_ASAN := y

subdir :=
subdir += common
subdir += communicate
# subdir += Transport
# subdir += Record

ifeq ($(MODULE_SYSTEM), y)
	CFLAGS_EX += -DMODULE_SYSTEM
	subdir += System
endif
ifeq ($(MODULE_VPP), y)
	CFLAGS_EX += -DMODULE_VPP
	subdir += Platform/$(PLATFORM)
endif
ifeq ($(MODULE_NETWORK), y)
	CFLAGS_EX += -DMODULE_NETWORK
	subdir += Network
endif
ifeq ($(MODULE_RECORD), y)
	CFLAGS_EX += -DMODULE_RECORD
endif
ifeq ($(MODULE_ALARM), y)
	CFLAGS_EX += -DMODULE_ALARM
	subdir += Alarm
endif
ifeq ($(MODULE_RTSP), y)
	CFLAGS_EX += -DMODULE_RTSP
	subdir += Transport/rtspserver/live555
	subdir += Transport/rtspserver
endif
ifeq ($(MODULE_WEBSOCKET), y)
	CFLAGS_EX += -DMODULE_WEBSOCKET
	subdir += Transport/websocket
endif
ifeq ($(MODULE_MEDIA_SERVER), y)
	CFLAGS_EX += -DMODULE_MEDIA_SERVER
	subdir += WebServer/
	subdir += Transport/media_server/mk_api
	subdir += Transport/media_server
endif
subdir += main

############################################################
ifeq ($(MODULE_VPP), y)
	PLATFORM_LIBS_NAME := cam vpf hbmem multimedia avformat avcodec avutil swresample ffmedia gdcbin cjson alog dnn hbucp ssl crypto drm z dl rt pthread mk_api jsoncpp zlmediakit zltoolkit mov ext-codec mpeg flv
	PLATFORM_LIBS += $(patsubst %,-l%,$(PLATFORM_LIBS_NAME))
	LDFLAGS_EX += -L$(HBRE_LIB)
endif

ifeq ($(MODULE_ENABLE_ASAN), y)
	PLATFORM_LIBS += -lasan
	CFLAGS_EX += -fsanitize=address -static-libasan -lasan
endif

GLOBAL_EXTERN_INC_DIR += $(HBRE_INC)
