################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
Archive/amp_dac.obj: ../Archive/amp_dac.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"/Applications/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.2.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="/Users/thomaskrause/workspace_v8/AMP_control_coolterm" --include_path="/Users/TK/Purdue/AMP/v210/F2837xD_headers/include" --include_path="/Users/TK/Purdue/AMP/v210/F2837xD_common/include" --include_path="/Applications/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.2.LTS/include" --define=CPU1 -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="Archive/amp_dac.d_raw" --obj_directory="Archive" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Archive/amp_pwm.obj: ../Archive/amp_pwm.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"/Applications/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.2.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="/Users/thomaskrause/workspace_v8/AMP_control_coolterm" --include_path="/Users/TK/Purdue/AMP/v210/F2837xD_headers/include" --include_path="/Users/TK/Purdue/AMP/v210/F2837xD_common/include" --include_path="/Applications/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.2.LTS/include" --define=CPU1 -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="Archive/amp_pwm.d_raw" --obj_directory="Archive" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Archive/amp_serial.obj: ../Archive/amp_serial.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"/Applications/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.2.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="/Users/thomaskrause/workspace_v8/AMP_control_coolterm" --include_path="/Users/TK/Purdue/AMP/v210/F2837xD_headers/include" --include_path="/Users/TK/Purdue/AMP/v210/F2837xD_common/include" --include_path="/Applications/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.2.LTS/include" --define=CPU1 -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="Archive/amp_serial.d_raw" --obj_directory="Archive" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Archive/amp_service.obj: ../Archive/amp_service.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"/Applications/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.2.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="/Users/thomaskrause/workspace_v8/AMP_control_coolterm" --include_path="/Users/TK/Purdue/AMP/v210/F2837xD_headers/include" --include_path="/Users/TK/Purdue/AMP/v210/F2837xD_common/include" --include_path="/Applications/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.2.LTS/include" --define=CPU1 -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="Archive/amp_service.d_raw" --obj_directory="Archive" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


