# uAlberta Biomed Makefile 
# 
# Requires configuration based on user operating system
# and installation file paths 
# Arduino Make file. Refer to https://github.com/sudar/Arduino-Makefile

# Path to arduino IDE 1.x.x installation folder 
ARDUINO_DIR   = D:/Arduino
BOARD_TXT	= D:/Arduino/hardware/arduino/avr
MONITOR_PORT	= com5


# Used for other arduino models (use `make show_submenu`)
# BOARD_SUB 		= 

#
# Common
#

ARDMK_DIR 		= ../../../Projects/Arduino-Makefile
BOARD_TAG 		= uno
ARCHITECTURE	= avr
USER_LIB_PATH	= ../lib
OBJDIR			= ../bin

# Windows monitor
MONITOR_CMD   = putty
MONITOR_PARAMS = 8,1,n,N

# Space seperated list of included libraries
ARDUINO_LIBS	= SoftwareSerial ODrive/Arduino/ODriveArduino
# From sudar's repo, main Makefile that compiles and uploads to Arduino
include $(ARDMK_DIR)/Arduino.mk