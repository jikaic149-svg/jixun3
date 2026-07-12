TARGET = spi_bmi088
BUILD_DIR = build

ifeq ($(OS),Windows_NT)
SHELL := C:/Windows/System32/cmd.exe
.SHELLFLAGS := /C
endif

PREFIX ?= arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size

HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

MCU = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

C_DEFS = \
-DUSE_HAL_DRIVER \
-DSTM32F407xx \
-DBMI088_USE_SPI

C_INCLUDES = \
-IInc \
-IDrivers/STM32F4xx_HAL_Driver/Inc \
-IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
-IDrivers/CMSIS/Include \
-Ibsp \
-Ibsp/boards \
-IDevice/bmi088 \
-IDevice/bluetooth \
-IDevice/vision \
-IDevice/track \
-IDevice/vofa \
-Imode \
-IApplication/algorithm \
-IApplication/ins \
-IApplication/gimbal \
-IMiddlewares/Third_Party/FreeRTOS/Source/include \
-IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS \
-I$(FREERTOS_PORT_DIR)

FREERTOS_PORT_DIR ?= Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F

C_SOURCES = \
Src/main.c \
Src/gpio.c \
Src/dma.c \
Src/spi.c \
Src/tim.c \
Src/can.c \
Src/freertos.c \
Src/stm32f4xx_it.c \
Src/stm32f4xx_hal_msp.c \
Src/system_stm32f4xx.c \
bsp/bsp_spi.c \
bsp/bsp_uart.c \
bsp/bsp_uart6.c \
bsp/bsp_delay.c \
bsp/bsp_laser.c \
bsp/bsp_can.c \
Device/bmi088/bmi088.c \
Device/bluetooth/bluetooth.c \
Device/vision/vision.c \
Device/track/track.c \
Device/vofa/vofa.c \
mode/mode_control.c \
mode/mode_stop.c \
mode/mode_forward.c \
mode/mode_backward.c \
mode/mode_turn_left.c \
mode/mode_turn_right.c \
mode/mode_tracking.c \
mode/mode_square_tracking.c \
mode/mode_gimbal.c \
Application/algorithm/attitude_algorithm.c \
Application/algorithm/crc16.c \
Application/algorithm/adrc_eso.c \
Application/algorithm/lqr_controller.c \
Application/algorithm/balance_outer_loop.c \
Application/algorithm/balance_zero_bias_search.c \
Application/algorithm/balance_control_helpers.c \
Application/ins/ins_task.c \
Application/gimbal/chassis_serial_cmd.c \
Application/gimbal/gimbal_task.c \
Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
Middlewares/Third_Party/FreeRTOS/Source/queue.c \
Middlewares/Third_Party/FreeRTOS/Source/list.c \
Middlewares/Third_Party/FreeRTOS/Source/timers.c \
Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c \
Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
$(FREERTOS_PORT_DIR)/port.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c

ASM_SOURCES = MDK-ARM/startup_stm32f407xx_gcc.s
LDSCRIPT = STM32F407IGHx_FLASH.ld

OPT = -Og

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -g -gdwarf-2 -MMD -MP
ASFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -g -gdwarf-2
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -lc -lm -lnosys

source_to_object = $(BUILD_DIR)/$(subst /,_,$(basename $(1))).o
source_to_list = $(BUILD_DIR)/$(subst /,_,$(basename $(1))).lst

OBJECTS = $(foreach src,$(C_SOURCES),$(call source_to_object,$(src)))
OBJECTS += $(foreach src,$(ASM_SOURCES),$(call source_to_object,$(src)))
DEPS = $(OBJECTS:.o=.d)

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

define COMPILE_C_RULE
$(call source_to_object,$(1)): $(1) Makefile | $(BUILD_DIR)
	$$(CC) -c $$(CFLAGS) -Wa,-a,-ad,-alms=$(call source_to_list,$(1)) $$< -o $$@
endef

define COMPILE_ASM_RULE
$(call source_to_object,$(1)): $(1) Makefile | $(BUILD_DIR)
	$$(AS) -c $$(ASFLAGS) $$< -o $$@
endef

$(foreach src,$(C_SOURCES),$(eval $(call COMPILE_C_RULE,$(src))))
$(foreach src,$(ASM_SOURCES),$(eval $(call COMPILE_ASM_RULE,$(src))))

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir $(BUILD_DIR)

clean:
	-rmdir /S /Q $(BUILD_DIR)

.PHONY: all clean

-include $(DEPS)
