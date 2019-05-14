################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/Users/Kai Strubel/Documents/Extra Curricular/AMP/TI/ti_code/AMP_pwm_control" --include_path="/Users/TK/Purdue/AMP/v210/F2837xD_common/include" --include_path="/Users/TK/Purdue/AMP/v210/F2837xD_headers/include" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/include" --advice:performance=all --define=CPU1 -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

%.obj: ../%.asm $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/Users/Kai Strubel/Documents/Extra Curricular/AMP/TI/ti_code/AMP_pwm_control" --include_path="/Users/TK/Purdue/AMP/v210/F2837xD_common/include" --include_path="/Users/TK/Purdue/AMP/v210/F2837xD_headers/include" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.4.LTS/include" --advice:performance=all --define=CPU1 -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


