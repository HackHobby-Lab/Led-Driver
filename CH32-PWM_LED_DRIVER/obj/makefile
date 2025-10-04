################################################################################
# MRS Version: 2.1.0
# Automatically-generated file. Do not edit!
################################################################################
-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include User/subdir.mk
-include Startup/subdir.mk
-include Peripheral/src/subdir.mk
-include Debug/subdir.mk
-include Core/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
CH32-PWM_LED_DRIVER.hex \

SECONDARY_LIST += \
CH32-PWM_LED_DRIVER.lst \

SECONDARY_SIZE += \
CH32-PWM_LED_DRIVER.siz \


# All Target
all: 
	$(MAKE) --no-print-directory main-build 

main-build: CH32-PWM_LED_DRIVER.elf secondary-outputs

# Tool invocations
CH32-PWM_LED_DRIVER.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU RISC-V Cross C Linker'
	riscv-none-embed-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -T "e:/CH32-RISCV/CH32-PWM_LED_DRIVER/Ld/Link.ld" -nostartfiles -Xlinker --gc-sections -Wl,-Map,"CH32-PWM_LED_DRIVER.map" --specs=nano.specs --specs=nosys.specs -o "CH32-PWM_LED_DRIVER.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@
CH32-PWM_LED_DRIVER.hex: CH32-PWM_LED_DRIVER.elf
	@echo 'Invoking: GNU RISC-V Cross Create Flash Image'
	@	riscv-none-embed-objcopy -O ihex "CH32-PWM_LED_DRIVER.elf" "CH32-PWM_LED_DRIVER.hex"
	@echo 'Finished building $@'
	@
CH32-PWM_LED_DRIVER.lst: CH32-PWM_LED_DRIVER.elf
	@echo 'Invoking: GNU RISC-V Cross Create Listing'
	@	riscv-none-embed-objdump --all-headers --demangle --disassemble -M xw "CH32-PWM_LED_DRIVER.elf" > "CH32-PWM_LED_DRIVER.lst"
	@echo 'Finished building $@'
	@
CH32-PWM_LED_DRIVER.siz: CH32-PWM_LED_DRIVER.elf
	@echo 'Invoking: GNU RISC-V Cross Print Size'
	riscv-none-embed-size --format=berkeley "CH32-PWM_LED_DRIVER.elf"
	@echo 'Finished building $@'
	@

# Other Targets
clean:
	-$(RM) $(OBJS)$(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE)$(S_DEPS)$(S_UPPER_DEPS)$(ASM_DEPS)$(ASM_UPPER_DEPS)$(C_DEPS) CH32-PWM_LED_DRIVER.elf

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets