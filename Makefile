CYGWIN=c:\\cygwin
MCC18DIR=/opt/mcc18_2
MCC18DIRW=$(CYGWIN)\\opt\\mcc18_2
MCC18=$(MCC18DIR)/bin/mcc18
PIC=18F8722 # Don't change
ARGS=-D_FRC_BOARD -w1 pa=3 -O+ -nw=2066
LINKER=$(MCC18DIR)/bin/mplink
MP2HEX=$(MCC18DIR)/bin/mp2hex
LIB_PATH=$(MCC18DIRW)\\lib
INCLUDE_PATH=$(MCC18DIR)/h
INCLUDE_PATHW=$(MCC18DIRW)\\h


#THESE NEED UPDATED PER CODE THING
ALL_TARGETS=camera.o camera2.o ifi_startup.o ifi_utilities.o main.o pwm.o serial_ports.o terminal.o tracking.o tracking2.o user_routines.o user_routines_fast.o 
INCLUDE_FILES=camera.h camera2.h delays.h ifi_aliases.h ifi_default.h ifi_utilities.h pwm.h serial_ports.h terminal.h tracking.h tracking2.h user_routines.h 


############ Targets: 
default: FrcCode.hex

#Remove all the individual file backup (The files ending with ~)
#NOTE that this does NOT remove your tbz2 backups!
clean-backup-files:
	@echo -e "Removing all the ~ backup files your annoying editor makes..."
	@/bin/find . -name "*~" |xargs rm -f

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

all: clean FrcCode.hex
%.o: %.c
	@echo -e "Compiling $@..."
	@$(MCC18) -p=$(PIC) -fo $@ $< /i\"$(INCLUDE_PATHW)\" $(ARGS) 


FrcCode.hex :  $(ALL_TARGETS) $(INCLUDE_FILES)
	@echo -e "Linking..."
	@$(LINKER) /l"$(LIB_PATH)" "18f8722.lkr" $(ALL_TARGETS) "FRC_library_8722.lib" /m"FrcCode.map" /o"FrcCode.cof"
	$(MP2HEX) FrcCode.cof



.deps/*: 
	@echo -e "Creating Dependency Makefiles..."
	@mkdir -p .deps
	@touch .deps/dummy
	@echo -e "Done!"

include .deps/*
