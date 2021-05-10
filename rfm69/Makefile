PROJECT = rfm69
BUILD_DIR = bin

SHARED_DIR = ../my-common-code
CFILES = main.c
CFILES += RFM69.c
CFILES += gpio.c
CFILES += spi.c
CFILES += mcp9808.c
CFILES += timer.c
CFILES += i2c.c
CFILES += utils.c

# TODO - you will need to edit these two lines!
DEVICE=stm32l010f4t
# OOCD_FILE = board/stm32f4discovery.cfg
OOCD_TARGET = target/stm32l.cfg

# You shouldn't have to edit anything below here.
VPATH += $(SHARED_DIR)
INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))
OPENCM3_DIR=../libopencm3

include $(OPENCM3_DIR)/mk/genlink-config.mk
include ../rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk
