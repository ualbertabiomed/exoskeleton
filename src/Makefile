# uAlberta Biomed Makefile 
# 
# Requires configuration based on user operating system
# and installation file paths 

# Windows 

# MacOS
# ARDUINO_DIR = /Applications/Arduino.app/Contents/Java # Path to arduino IDE 1.x.x installation folder. Should be same if user followed guide.
# MONITOR_PORT = /dev/cu.usb*

# Linux 
ARDUINO_DIR		= /usr/share/arduino/arduino-1.8.9/arduino-1.8.9 # Path to arduino IDE 1.x.x installation folder 
BOARDS_TXT		= $(ARDUINO_DIR)/hardware/arduino/avr/boards.txt # Path to boards.txt file
MONITOR_PORT	= /dev/ttyACM*
ARCHITECTURE	= avr
# BOARD_SUB 		= # used for other arduino models (use `make show_submenu`)
# AVR_TOOLS_DIR	= # Path to AVR tools, makefile might detect this automatically. Arduino IDE has pre-installed avr tools

# Common
ARDMK_DIR 		= ../setup/Arduino-Makefile
BOARD_TAG 		= uno
# ARDUINO_LIBS	= # Space seperated list of included libraries
USER_LIB_PATH	= ../lib
OBJDIR			= ../bin

# From sudar's repo, main Makefile that compiles and uploads to Arduino
include $(ARDMK_DIR)/Arduino.mk