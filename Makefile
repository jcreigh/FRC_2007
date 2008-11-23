MCC18=/opt/mcc18_2/bin/mcc18
PIC=18F8722 # Don't change
ARGS=-D_FRC_BOARD -w1 pa=3 -O+ -nw=2066
LINKER=/opt/mcc18_2/bin/mplink
MP2HEX=/opt/mcc18_2/bin/mp2hex
IFILOAD=/Program\ Files/IFI_Loader/IFI_Loader.exe
#picloader_textmode
SERIAL_DEV=
#/dev/ttyS0
READLOG=./read.log
LIB_PATH=c:\\cygwin\\opt\\mcc18_2\\lib
CODE_PATH=c:\\code\\Charlotte\\
INCLUDE_PATH=/opt/mcc18_2/h
INCLUDE_PATHW=c:\\cygwin\\opt\\mcc18_2\\h




ALL_TARGETS=camera.o camera2.o ifi_startup.o ifi_utilities.o main.o pwm.o serial_ports.o terminal.o tracking.o tracking2.o user_routines.o user_routines_fast.o 
INCLUDE_FILES=camera.h camera2.h delays.h ifi_aliases.h ifi_default.h ifi_utilities.h pwm.h serial_ports.h terminal.h tracking.h tracking2.h user_routines.h 
############ Targets: 
default: FrcCode.hex

#Remove all the individual file backup (The files ending with ~)
#NOTE that this does NOT remove your tbz2 backups!
clean-backup-files:
	@echo -e "Removing all the ~ backup files your annoying editor makes..."
	@find . -name "*~" |xargs rm -f

#Removes all compiled files from the directory
clean: clean-backup-files clean-deps nohex
	@echo -e "Cleaning compiler intermediate files..."
	@rm -rf *.o *.err

#Remove deps folder
clean-deps:
	@echo -e "Cleaning dependency folder..."
	@rm -rf .deps
	@mkdir .deps
	@touch .deps/dummy

#Remove hexfiles
nohex	:
	@echo -e "Cleaning linker output and hex files..."
	@rm -rf *.hex *.cod *.cof

#These all start the IFI Loader.
safeload: prettycode clean check load
load: FrcCode.hex
	@$(IFILOAD) "FrcCode.hex"
	#$(SERIAL_DEV)

read:
	@echo -e "Monitoring $(SERIAL_DEV) for data"
	@echo -e "Output will also be logged to $(READLOG)"
	@echo -e "---------"
	@cat $(SERIAL_DEV) | tee $(READLOG) || /bin/true
    
    
prettycode:
	@echo -e "Using indent to autoformat your .c and .h files..."
	@indent *.c *.h
all: clean FrcCode.hex
%.o: %.c
	@echo -e "Compiling $@..."
	@echo -e "1.   Launching Compiler..."
	@$(MCC18) -p=$(PIC) -fo $@ $< /i\"$(INCLUDE_PATHW)\" $(ARGS) 
	@echo -e "Done with $@."


FrcCode.hex :  $(ALL_TARGETS) $(INCLUDE_FILES)
	@echo -e "Linking..."
	@$(LINKER) /l"$(LIB_PATH)" "18f8722.lkr" $(ALL_TARGETS) "$(CODE_PATH)FRC_library_8722.lib" /m"FrcCode.map" /o"FrcCode.cof"
	$(MP2HEX) FrcCode.cof



.deps/*: 
	@echo -e "Creating Dependency Makefiles..."
	@mkdir -p .deps
	@touch .deps/dummy
	@echo -e "Done!"

include .deps/*
