################################################################################
# Automatically-generated file. Do not edit!
################################################################################

ifdef SystemRoot
	SHELL = cmd.exe
	MK_DIR = mkdir
else
	ifeq ($(shell uname), Linux)
		MK_DIR = mkdir -p
	endif

	ifeq ($(shell uname | cut -d _ -f 1), CYGWIN)
		MK_DIR = mkdir -p
	endif

	ifeq ($(shell uname | cut -d _ -f 1), MINGW32)
		MK_DIR = mkdir -p
	endif

	ifeq ($(shell uname | cut -d _ -f 1), MINGW64)
		MK_DIR = mkdir -p
	endif

	ifeq ($(shell uname | cut -d _ -f 1), DARWIN)
		MK_DIR = mkdir -p
	endif
endif

# List the subdirectories for creating object files
SUB_DIRS +=  \
 \
atmel/hpl/ramecc \
atmel/hpl/rtc \
atmel/hpl/gmac \
atmel/examples \
atmel/hpl/systick \
atmel/hpl/tc \
atmel/hpl/oscctrl \
atmel/thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source \
atmel/thirdparty/RTOS/freertos/FreeRTOSV10.0.0 \
atmel/stdio_redirect/gcc \
atmel/gcc \
atmel/hpl/osc32kctrl \
atmel/hpl/dmac \
atmel/hal/src \
atmel/stdio_redirect \
atmel/hal/utils/src \
atmel/thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/portable/GCC/ARM_CM4F \
atmel/thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/portable/MemMang \
atmel/hpl/pm \
atmel/hpl/adc \
atmel/hpl/cmcc \
atmel/hpl/gclk \
atmel/gcc/gcc \
atmel/hpl/mclk \
atmel/hpl/sercom \
atmel/hpl/core

# List the object files
OBJS +=  \
atmel/hal/src/hal_io.o \
atmel/gcc/system_same53.o \
atmel/thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/list.o \
atmel/hpl/systick/hpl_systick.o \
atmel/hpl/tc/hpl_tc.o \
atmel/hpl/gmac/hpl_gmac.o \
atmel/thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/portable/GCC/ARM_CM4F/port.o \
atmel/thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/event_groups.o \
atmel/thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/croutine.o \
atmel/stdio_redirect/stdio_io.o \
atmel/stdio_redirect/gcc/write.o \
atmel/hal/src/hal_init.o \
atmel/hpl/core/hpl_core_m4.o \
atmel/thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/stream_buffer.o \
atmel/hal/utils/src/utils_syscalls.o \
atmel/thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/portable/MemMang/heap_1.o \
atmel/thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/queue.o \
atmel/hal/src/hal_spi_m_sync.o \
atmel/stdio_redirect/gcc/read.o \
atmel/thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/timers.o \
atmel/hal/src/hal_delay.o \
atmel/hpl/pm/hpl_pm.o \
atmel/hpl/core/hpl_init.o \
atmel/hpl/gclk/hpl_gclk.o \
atmel/hal/utils/src/utils_list.o \
atmel/hal/utils/src/utils_assert.o \
atmel/hpl/dmac/hpl_dmac.o \
atmel/hpl/oscctrl/hpl_oscctrl.o \
atmel/hal/src/hal_usart_sync.o \
atmel/hpl/mclk/hpl_mclk.o \
atmel/hpl/ramecc/hpl_ramecc.o \
atmel/stdio_start.o \
atmel/hal/src/hal_spi_m_async.o \
atmel/thirdparty/RTOS/freertos/FreeRTOSV10.0.0/rtos_port.o \
atmel/rtos_start.o \
atmel/hpl/osc32kctrl/hpl_osc32kctrl.o \
atmel/examples/driver_examples.o \
atmel/driver_init.o \
atmel/hpl/rtc/hpl_rtc.o \
atmel/hpl/sercom/hpl_sercom.o \
atmel/main.o \
atmel/gcc/gcc/startup_same53.o \
atmel/hal/src/hal_gpio.o \
atmel/hal/src/hal_mac_async.o \
atmel/hal/utils/src/utils_event.o \
atmel/hal/utils/src/utils_ringbuffer.o \
atmel/hal/src/hal_sleep.o \
atmel/hal/src/hal_cache.o \
atmel/hpl/adc/hpl_adc.o \
atmel/hpl/cmcc/hpl_cmcc.o \
atmel/atmel_start.o \
atmel/hal/src/hal_atomic.o \
atmel/hal/src/hal_adc_sync.o \
atmel/hal/src/hal_adc_async.o \
atmel/hal/src/hal_pwm.o \
atmel/hal/src/hal_timer.o \
atmel/thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/tasks.o

# List the directories containing header files
DIR_INCLUDES +=  \
-I"../src/atmel" \
-I"../src/atmel/config" \
-I"../src/atmel/examples" \
-I"../src/atmel/hal/include" \
-I"../src/atmel/hal/utils/include" \
-I"../src/atmel/hpl/cmcc" \
-I"../src/atmel/hpl/core" \
-I"../src/atmel/hpl/dmac" \
-I"../src/atmel/hpl/gclk" \
-I"../src/atmel/hpl/mclk" \
-I"../src/atmel/hpl/osc32kctrl" \
-I"../src/atmel/hpl/oscctrl" \
-I"../src/atmel/hpl/pm" \
-I"../src/atmel/hpl/port" \
-I"../src/atmel/hpl/ramecc" \
-I"../src/atmel/hpl/sercom" \
-I"../src/atmel/hpl/systick" \
-I"../src/atmel/hpl/tc" \
-I"../src/atmel/hpl/adc" \
-I"../src/atmel/hpl/rtc" \
-I"../src/atmel/hri" \
-I"../src/atmel/config" \
-I"../src/atmel/thirdparty/RTOS" \
-I"../src/atmel/thirdparty/RTOS/freertos/FreeRTOSV10.0.0" \
-I"../src/atmel/thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/include" \
-I"../src/atmel/thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/portable/GCC/ARM_CM4F" \
-I"../src/atmel/thirdparty/RTOS/freertos/FreeRTOSV10.0.0/module_config" \
-I"../src/atmel/config" \
-I"../src/atmel/stdio_redirect" \
-I"../src/atmel/CMSIS/Core/Include" \
-I"../src/atmel/include" \
-I"../src" \

# List the dependency files
DEPS := $(OBJS:%.o=%.d)

OUTPUT_FILE_NAME := e53n20a-demo
QUOTE := "
OUTPUT_FILE_PATH +=$(OUTPUT_FILE_NAME).elf
OUTPUT_FILE_PATH_AS_ARGS +=$(OUTPUT_FILE_NAME).elf

vpath %.c ../src
vpath %.s ../src
vpath %.S ../src

# All Target
all: $(SUB_DIRS) $(OUTPUT_FILE_PATH)

# Linker target

$(OUTPUT_FILE_PATH): $(OBJS)
	@echo Building target: $@
	@echo Invoking: ARM/GNU Linker
	$(QUOTE)arm-none-eabi-gcc$(QUOTE) -o $(OUTPUT_FILE_NAME).elf $(OBJS) -Wl,--start-group -lm -Wl,--end-group -mthumb \
-Wl,-Map="$(OUTPUT_FILE_NAME).map" --specs=nano.specs -Wl,--gc-sections -mcpu=cortex-m4 \
 \
-T"same53n20a_flash.ld" \
-L"../gcc/gcc"
	@echo Finished building target: $@

	"arm-none-eabi-objcopy" -O binary "$(OUTPUT_FILE_NAME).elf" "$(OUTPUT_FILE_NAME).bin"
	"arm-none-eabi-objcopy" -O ihex -R .eeprom -R .fuse -R .lock -R .signature  \
        "$(OUTPUT_FILE_NAME).elf" "$(OUTPUT_FILE_NAME).hex"
	"arm-none-eabi-objcopy" -j .eeprom --set-section-flags=.eeprom=alloc,load --change-section-lma \
        .eeprom=0 --no-change-warnings -O binary "$(OUTPUT_FILE_NAME).elf" \
        "$(OUTPUT_FILE_NAME).eep" || exit 0
	"arm-none-eabi-objdump" -h -S "$(OUTPUT_FILE_NAME).elf" > "$(OUTPUT_FILE_NAME).lss"
	"arm-none-eabi-size" "$(OUTPUT_FILE_NAME).elf"

	

# Compiler targets




%.o: %.c
	@echo Building file: $<
	@echo ARM/GNU C Compiler
	$(QUOTE)arm-none-eabi-gcc$(QUOTE) -x c -mthumb -DDEBUG -Os -ffunction-sections -mlong-calls -g3 -Wall -c -std=gnu99 \
-D__SAME53N20A__ -mcpu=cortex-m4 -mfloat-abi=softfp -mfpu=fpv4-sp-d16 \
$(DIR_INCLUDES) \
-MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"  -o "$@" "$<"
	@echo Finished building: $<

%.o: %.s
	@echo Building file: $<
	@echo ARM/GNU Assembler
	$(QUOTE)arm-none-eabi-as$(QUOTE) -x c -mthumb -DDEBUG -Os -ffunction-sections -mlong-calls -g3 -Wall -c -std=gnu99 \
-D__SAME53N20A__ -mcpu=cortex-m4 -mfloat-abi=softfp -mfpu=fpv4-sp-d16 \
$(DIR_INCLUDES) \
-MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"  -o "$@" "$<"
	@echo Finished building: $<

%.o: %.S
	@echo Building file: $<
	@echo ARM/GNU Preprocessing Assembler
	$(QUOTE)arm-none-eabi-gcc$(QUOTE) -x c -mthumb -DDEBUG -Os -ffunction-sections -mlong-calls -g3 -Wall -c -std=gnu99 \
-D__SAME53N20A__ -mcpu=cortex-m4 -mfloat-abi=softfp -mfpu=fpv4-sp-d16 \
$(DIR_INCLUDES) \
-MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"  -o "$@" "$<"
	@echo Finished building: $<

# Detect changes in the dependent files and recompile the respective object files.
ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(DEPS)),)
-include $(DEPS)
endif
endif

$(SUB_DIRS):
	$(MK_DIR) "$@"

clean:
	rm -f $(OBJS)
	rm -f $(OUTPUT_FILE_PATH)
	rm -f $(DEPS)
	rm -f $(OUTPUT_FILE_NAME).a $(OUTPUT_FILE_NAME).hex $(OUTPUT_FILE_NAME).bin \
        $(OUTPUT_FILE_NAME).lss $(OUTPUT_FILE_NAME).eep $(OUTPUT_FILE_NAME).map \
        $(OUTPUT_FILE_NAME).srec

tags:
	(cd ..; ctags -R .)

.PHONY: tags all clean
