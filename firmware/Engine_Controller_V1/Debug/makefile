################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM :=  -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include system/src/newlib/subdir.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: Engine_Controller_V1

# Tool invocations
	@echo 'No tool found that can build the extension specified with the build artifact name $@'
# Other Targets
clean:
	-$(RM) $(OBJS) Engine_Controller_V1
	-@echo ' '

post-build:
	-rm-none-eabi-objcopy -O ihex “Engine_Controller_V1.elf” “Engine_Controller_V1.hex”
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
