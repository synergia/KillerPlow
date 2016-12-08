################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../accelerometer/acceleroeter.c 

OBJS += \
./accelerometer/acceleroeter.o 

C_DEPS += \
./accelerometer/acceleroeter.d 


# Each subdirectory must supply rules for building sources it contributes
accelerometer/%.o: ../accelerometer/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega88pa -DF_CPU=20000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


