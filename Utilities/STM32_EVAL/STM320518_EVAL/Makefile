CC=arm-none-eabi-gcc
AR=arm-none-eabi-ar

###########################################

CFLAGS  = -g -O2 -Wall
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m0 -march=armv6s-m
CFLAGS += -ffreestanding -nostdlib
CFLAGS += -include../../../Libraries/stm32f0xx_conf.h -I../../../Libraries/CMSIS/Include -I../../../Libraries/CMSIS/Device/ST/STM32F0xx/Include -I../../../Libraries/STM32F0xx_StdPeriph_Driver/inc/ -I. -I ../Common/ -I../../../Libraries/STM32F0xx_CPAL_Driver/inc


SRCS =	stm320518_eval.c			\
	stm320518_eval_cec.c			\
	stm320518_eval_i2c_ee.c			\
	stm320518_eval_i2c_tsensor.c		\
	stm320518_eval_lcd.c			\
	stm320518_eval_spi_sd.c			\


OBJS = $(SRCS:.c=.o)

.PHONY: libstm320518_eval.a

all: libstm320518_eval.a

%.o : %.c
	$(CC) $(CFLAGS) -c -o $@ $^

libstm320518_eval.a: $(OBJS)
	$(AR) -r $@ $(OBJS)

clean:
	rm -f $(OBJS) libstm320518_eval.a
