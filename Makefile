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
-IDevice/vofa \
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
bsp/bsp_can.c \
Device/bmi088/bmi088.c \
Device/bluetooth/bluetooth.c \
Device/vision/vision.c \
Device/vofa/vofa.c \
Application/algorithm/attitude_algorithm.c \
Application/algorithm/crc16.c \
Application/algorithm/adrc_eso.c \
Application/algorithm/lqr_controller.c \
Application/algorithm/balance_outer_loop.c \
Application/algorithm/balance_zero_bias_search.c \
Application/ins/ins_task.c \
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

OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))
DEPS = $(OBJECTS:.o=.d)

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(ASFLAGS) $< -o $@

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
