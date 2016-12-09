################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Code/accelerometer/subdir.mk
-include Code/Release/accelerometer/subdir.mk
-include Code/Release/MKUART/subdir.mk
-include Code/Release/I2C_TWI/subdir.mk
-include Code/Release/subdir.mk
-include Code/MKUART/subdir.mk
-include Code/I2C_TWI/subdir.mk
-include Code/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
KillerPlow.lss \

FLASH_IMAGE += \
KillerPlow.hex \

EEPROM_IMAGE += \
KillerPlow.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: KillerPlow.elf secondary-outputs

# Tool invocations
KillerPlow.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,KillerPlow.map -mmcu=atmega88p -o "KillerPlow.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

KillerPlow.lss: KillerPlow.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S KillerPlow.elf  >"KillerPlow.lss"
	@echo 'Finished building: $@'
	@echo ' '

KillerPlow.hex: KillerPlow.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex KillerPlow.elf  "KillerPlow.hex"
	@echo 'Finished building: $@'
	@echo ' '

KillerPlow.eep: KillerPlow.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex KillerPlow.elf  "KillerPlow.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: KillerPlow.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega88p KillerPlow.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(FLASH_IMAGE)$(ELFS)$(OBJS)$(ASM_DEPS)$(EEPROM_IMAGE)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(LSS)$(C_DEPS) KillerPlow.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
