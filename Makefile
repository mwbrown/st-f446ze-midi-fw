
# Check for the presence of STM32CUBE_F4
ifndef STM32CUBE_F4
  $(error STM32CUBE_F4 must be set to the STM32Cube installation.)
endif

# Toolchain Configuration
PREFIX     := arm-none-eabi-
CC         := $(PREFIX)gcc
OBJCOPY    := $(PREFIX)objcopy
OBJDUMP    := $(PREFIX)objdump
GDB        := $(PREFIX)gdb

# Source Directories
CMSIS_DIR  := $(STM32CUBE_F4)/Drivers/CMSIS
HAL_DIR    := $(STM32CUBE_F4)/Drivers/STM32F4xx_HAL_Driver
BSP_DIR    := $(STM32CUBE_F4)/Drivers/BSP/STM32F4xx_Nucleo_144
COMP_DIR   := $(STM32CUBE_F4)/Drivers/BSP/Components

# Build variables
OBJDIR     := build
IMAGE_NAME := f446ze-midi

INCLUDES :=                                          \
	-Isrc                                            \
	-I$(CMSIS_DIR)/Include                           \
	-I$(CMSIS_DIR)/Device/ST/STM32F4xx/Include       \
	-I$(HAL_DIR)/Inc                                 \
	-I$(BSP_DIR)

DEFINES := -DSTM32F446xx -DUSE_FULL_LL_DRIVER -DHSE_VALUE=8000000U

# FIXME add FPU support
CPUFLAGS := -mcpu=cortex-m4 -mthumb

CFLAGS  = $(CPUFLAGS) -g -Os -ffreestanding -ffunction-sections -fdata-sections -Wall $(INCLUDES) $(DEFINES) 
LDFLAGS = $(CPUFLAGS) -Wl,-Map=$(IMAGE_NAME).map -T STM32F446ZETx_FLASH.ld -specs=nano.specs -Wl,--gc-sections

# Make implicit function declarations error out.
CFLAGS += -Werror=implicit-function-declaration

APP_SOURCES :=                  \
	src/main.c                  \
	src/stm32f4xx_it.c          \
	src/system_stm32f4xx.c

LL_SOURCES :=                             \
	$(HAL_DIR)/Src/stm32f4xx_ll_adc.c     \
	$(HAL_DIR)/Src/stm32f4xx_ll_crc.c     \
	$(HAL_DIR)/Src/stm32f4xx_ll_dac.c     \
	$(HAL_DIR)/Src/stm32f4xx_ll_dma.c     \
	$(HAL_DIR)/Src/stm32f4xx_ll_dma2d.c   \
	$(HAL_DIR)/Src/stm32f4xx_ll_exti.c    \
	$(HAL_DIR)/Src/stm32f4xx_ll_fmc.c     \
	$(HAL_DIR)/Src/stm32f4xx_ll_fsmc.c    \
	$(HAL_DIR)/Src/stm32f4xx_ll_gpio.c    \
	$(HAL_DIR)/Src/stm32f4xx_ll_i2c.c     \
	$(HAL_DIR)/Src/stm32f4xx_ll_lptim.c   \
	$(HAL_DIR)/Src/stm32f4xx_ll_pwr.c     \
	$(HAL_DIR)/Src/stm32f4xx_ll_rcc.c     \
	$(HAL_DIR)/Src/stm32f4xx_ll_rng.c     \
	$(HAL_DIR)/Src/stm32f4xx_ll_rtc.c     \
	$(HAL_DIR)/Src/stm32f4xx_ll_sdmmc.c   \
	$(HAL_DIR)/Src/stm32f4xx_ll_spi.c     \
	$(HAL_DIR)/Src/stm32f4xx_ll_tim.c     \
	$(HAL_DIR)/Src/stm32f4xx_ll_usart.c   \
	$(HAL_DIR)/Src/stm32f4xx_ll_usb.c     \
	$(HAL_DIR)/Src/stm32f4xx_ll_utils.c

STARTUP_SOURCES :=              \
	src/startup_stm32f446xx.s

SOURCES_C := $(APP_SOURCES) $(LL_SOURCES)
SOURCES_S := $(STARTUP_SOURCES)

OBJECTS_C := $(addprefix $(OBJDIR)/, $(SOURCES_C:.c=.c.o))
OBJECTS_S := $(addprefix $(OBJDIR)/, $(SOURCES_S:.s=.s.o))

OBJECTS := $(OBJECTS_C) $(OBJECTS_S)

all: $(IMAGE_NAME).hex

.PHONY: clean flash disasm debug

# Mark the objects as precious to enable fast rebuilds.
.PRECIOUS: $(OBJDIR)/%.o

#
# Phony Targets
#

clean:
	rm -f $(IMAGE_NAME).elf $(IMAGE_NAME).hex $(IMAGE_NAME).bin $(IMAGE_NAME).map
	rm -rf $(OBJDIR)/

flash: $(IMAGE_NAME).hex
	st-flash --reset --format ihex write $(IMAGE_NAME).hex

disasm: $(IMAGE_NAME).elf
	$(OBJDUMP) -d $< > $(IMAGE_NAME).disasm

debug: $(IMAGE_NAME).elf
	$(GDB) $<

#
# Top-level Targets
#

$(IMAGE_NAME).elf: $(OBJECTS)
	$(CC) $(LDFLAGS) -o $@ $^

#
# Pattern Rules
#

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

%.hex: %.elf
	$(OBJCOPY) -O ihex $< $@

-include $(OBJECTS_C:.o=.d)
-include $(OBJECTS_S:.o=.d)

$(OBJDIR)/%.c.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c -o $@ $<
	@$(CC) $(CFLAGS) -MM -MT $@ $< > $(OBJDIR)/$*.d

$(OBJDIR)/%.s.o: %.s
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c -o $@ $<
	@$(CC) $(CFLAGS) -MM -MT $@ $< > $(OBJDIR)/$*.d