#.SILENT:

PROJECT=fifisdr

ifeq ($(RTOS_BRAND),FREERTOS)
RTOS=FreeRTOS-7.1.0
RTOS_INCLUDE= \
    -I$(PDL)/libs/rtos/$(RTOS) \
    -I$(PDL)/libs/rtos/$(RTOS)/src/include \
    -I$(PDL)/libs/rtos/$(RTOS)/src/portable/GCC/ARM_CM3
RTOS_SOURCEDIR= \
    $(PDL)/libs/rtos/$(RTOS) \
    $(PDL)/libs/rtos/$(RTOS)/src \
    $(PDL)/libs/rtos/$(RTOS)/src/portable/MemMang \
    $(PDL)/libs/rtos/$(RTOS)/src/portable/GCC/ARM_CM3
RTOS_SRCS= \
    cmsis_os.c croutine.c list.c queue.c tasks.c timers.c heap_2.c port.c
RTOS_CFLAGS= -DRTOS_FREERTOS
RTOS_LDFLAGS=
endif
ifeq ($(RTOS_BRAND),RTX)
RTOS=RTX-4.20
RTOS_INCLUDE= \
    -I$(PDL)/libs/rtos/$(RTOS)
RTOS_SOURCEDIR= .
RTOS_SRCS= \
    RTX_Conf_CM.c
RTOS_CFLAGS= -DRTOS_RTX
RTOS_LDFLAGS= -Wl,-L$(PDL)/libs/rtos/$(RTOS)/GCC -Wl,-lRTX_CM3
endif


export CC=arm-none-eabi-gcc
export AR=arm-none-eabi-ar
export OBJCOPY=arm-none-eabi-objcopy
export OBJDUMP=arm-none-eabi-objdump
ARCH=arm-none-eabi-ar
export WARNINGS= \
    -Wextra -Wshadow -Wpointer-arith -Wbad-function-cast -Wcast-align -Wsign-compare -Wswitch \
    -Wstrict-prototypes -Wmissing-prototypes -Wmissing-declarations -Wunused \
    -Wall \
    -Wno-packed-bitfield-compat #-Wpacked
export LDSCRIPT=lpc1768-rom.ld
export PDL=$(shell pwd)/../pdl
export OBJDIR=obj
export CFLAGS=$(WARNINGS) $(INCLUDES) $(BASEINCLUDE) \
    -mcpu=cortex-m3 -mthumb -T$(LDSCRIPT) \
    -g -O2 -fomit-frame-pointer -ffunction-sections -fshort-wchar \
    -DSVNMAXREV=$(GIT_NEAREST_TAG) -DSVNNAME=$(GITNAME) $(RTOS_CFLAGS)
export LDFLAGS= \
    -nostartfiles $(RTOS_LDFLAGS) \
    -Wl,-Map=$(OBJDIR)/$(PROJECT).map,--cref,--gc-sections,-o$@,--no-wchar-size-warning,--print-memory-usage
export BASEINCLUDE= \
    -I../src \
    -I../src/bsp \
    -I../src/dsp \
    -I../src/gui \
    -I$(PDL)/drivers/lpc1700/inc \
    -I$(PDL)/core/Device/NXP/LPC17xx/Include \
    -I$(PDL)/core/CMSIS/Include \
    -I$(PDL)/devices/inc \
    $(RTOS_INCLUDE) \
    -I$(PDL)/libs/usb/dqsb/inc



vpath %.c ../src
vpath %   ../src/bsp
vpath %   ../src/dsp
vpath %   ../src/gui
vpath %   $(PDL)/drivers/lpc1700/src
vpath %   $(PDL)/core/Device/NXP/LPC17xx/Source
vpath %   $(PDL)/core/Device/NXP/LPC17xx/Source/GCC
vpath %   $(PDL)/devices/src
vpath %   $(PDL)/libs/usb/dqsb/src
vpath %   $(RTOS_SOURCEDIR)


SRCS = \
    i2cdev.c \
    si570.c st7565r.c \
    \
    lpc17xx_adc.c lpc17xx_clkpwr.c lpc17xx_dac.c lpc17xx_dma.c lpc17xx_gpio.c lpc17xx_i2c.c \
    lpc17xx_i2s.c lpc17xx_iocon.c lpc17xx_ssp.c lpc17xx_timer.c lpc17xx_usb.c \
    \
    usbuser_descriptors.c usbuser.c softrock.c params.c fifiaudio.c \
    fifidsp.c am.c ssb.c fm.c wbfm.c dc.c \
    \
    usbcore_device.c usbclass_audio.c \
    \
    $(RTOS_SRCS) \
    \
    bsp-fifisdr.c bsp-codec.c bsp-lcd.c bsp-touch.c \
    task-gui.c font-7seg.c task-sys.c \
    main.c \
    \
    startup_LPC17xx.c \
    system_LPC17xx.c


ASMSRCS = \
    hfa.s channel_filter.s channel_filter_fm.s nf_filter_fm.s interpol4.s

#
# Define all object files.
#
C_OBJ = $(patsubst %,$(OBJDIR)/%,$(SRCS:.c=.o))
ASM_OBJ = $(patsubst %,$(OBJDIR)/%,$(ASMSRCS:.s=.o))

.PHONY: all
all : $(OBJDIR)/$(PROJECT).hex

$(OBJDIR)/$(PROJECT).hex : Makefile $(OBJDIR)/$(PROJECT).axf
	@$(OBJCOPY) $(OBJDIR)/$(PROJECT).axf -O ihex $(OBJDIR)/$(PROJECT).hex
	@$(OBJCOPY) $(OBJDIR)/$(PROJECT).axf -O binary $(OBJDIR)/$(PROJECT).bin

$(OBJDIR)/$(PROJECT).axf : Makefile $(C_OBJ) $(ASM_OBJ) $(LDSCRIPT)
	@echo "Linking into $@..."
	@$(CC) $(CFLAGS) $(C_OBJ) $(ASM_OBJ) $(LDFLAGS)
	@$(OBJDUMP) -d -S $@ >$(OBJDIR)/$(PROJECT).lst

$(C_OBJ) : $(OBJDIR)/%.o : %.c Makefile
	@echo $(notdir $<)
	@mkdir -p $(dir $@)
	@$(CC) -c $(CFLAGS) $< -o $@

$(ASM_OBJ) : $(OBJDIR)/%.o : %.s Makefile
	@echo $(notdir $<)
	@mkdir -p $(dir $@)
	@$(CC) -c $(CFLAGS) $< -o $@

#
#  The .depend files contains the list of header files that the
#  various source files depend on.  By doing this, we'll only
#  rebuild the .o's that are affected by header files changing.
#
.depend: $(SRCS)
	@echo "Build dependencies..."
	@$(CC) $(CFLAGS) -M $+ | sed 's/^\([a-zA-Z0-9_.o]\)/$(OBJDIR)\/\1/' > .depend

-include .depend

#
#  Utility targets
#
.PHONY: tags
tags :
	@rm -f ctags
	@find . -name \*.c -exec ctags -a {} \;
	@find . -name \*.h -exec ctags -a {} \;

.PHONY: clean
clean :
	@find . -name \*.o -exec rm -f {} \;
	@find . -name .depend -exec rm -f {} \;
	@find . -name \*.map -exec rm -f {} \;
	@find . -name \*.lst -exec rm -f {} \;
	@find . -name \*.axf -exec rm -f {} \;
	@find . -name \*.elf -exec rm -f {} \;
	@find . -name \*.hex -exec rm -f {} \;
	@find . -name \*.bin -exec rm -f {} \;


