# uAlberta Biomed Makefile 
# 
# Requires configuration based on user operating system
# and installation file paths 

#
# Windows 
#

#
# MacOS
#

# Path to arduino IDE 1.x.x installation folder. Should be same if user followed guide.
# ARDUINO_DIR = /Applications/Arduino.app/Contents/Java 
# MONITOR_PORT = /dev/cu.usb*

# 
# Linux 
# 

# Path to arduino IDE 1.x.x installation folder 
ARDUINO_DIR		= /usr/share/arduino/arduino-1.8.9/arduino-1.8.9
MONITOR_PORT	= /dev/ttyACM*


# Used for other arduino models (use `make show_submenu`)
# BOARD_SUB 		= 

#
# Common
#

ARDMK_DIR 		= ../setup
BOARD_TAG 		= uno
ARCHITECTURE	= avr
USER_LIB_PATH	= ../lib
OBJDIR			= ../bin

# Space seperated list of included libraries
# ARDUINO_LIBS	= 
# From sudar's repo, main Makefile that compiles and uploads to Arduino
include $(ARDMK_DIR)/Arduino.mk