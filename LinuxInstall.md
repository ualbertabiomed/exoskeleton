# Fresh Install - Linux 
- [Github Link](https://github.com/sudar/Arduino-Makefile)

## Step 1: Installing arduino-mk
### Package Manager
There are two options. For the basic necessities of the package, use `sudo apt-get install arduino-mk`.  
- This will install it in /usr/share and create an arduino/ directory 
- Inside the new /usr/share/arduino directory the following files will be installed:
    - Arduino.mk
    - arduino-mk-vars.md
    - chipKIT.mk
    - Common.mk
    - Teensy.mk 
- **Do not edit these files**

### From Source
- But to get *everything* from the Github repo, you need to clone the entire repo into a directory (I clone it to /usr/share/arduino)
    - I manually created an arduino/ directory in /usr/share  
- Cloning will provide examples and other files (I do this method)
- `sudo git clone https://github.com/sudar/Arduino-Makefile.git` (I had to use sudo to clone it into /usr/share/arduino)

## Step 2: Installing the Arduino IDE
- As of July 6, 2019 the latest version is: Arduino 1.8.9
- [Linux Installation Guide](https://www.arduino.cc/en/Guide/Linux)
1) Download the tar.xz file and extract
2) Before installing the IDE, best to move the extracted folder (arduino-1.8.9-linux64) to either /opt or /usr/local or /usr/share/arduino 
    - I move it to /usr/share/arduino so it's in the same directory as arduino-mk
    - *Tip* to those who are unfamiliar with moving files into the root folders:
        - You need to use the terminal along with the `sudo` command 
        - When only changing directories, from home you can just go `cd /usr/share` but anywhere else or if you are moving 
        files / folders you need go (example is cd'ing from Downloads) `cd ../../../usr/share` 
3) run the instal.sh file in the extracted folder
    - You may need to first run `chmod +x install.sh` 
    - Then run `sudo ./install.sh` 

## Step 3: Installing pyserial
- Best to just follow the instructions outlined in the Github README

## Step 4: Configuring your Environment 
**Review:** By this step you **need** to ensure you have the following:
    - arduino-mk installed (either through apt-get *or* by cloning the repo), 
    - installed the latest version of the Arduino IDE 
    - installed pyserial 
    - know the path files to where you installed arduino-mk *and* the Arduino IDE 
        - I stored both arduino-mk and Arduino IDE in /usr/share/arduino 

**Sample Makefile** 
```
ARDUINO_DIR = /usr/share/arduino/arduino-1.8.9/arduino-1.8.9
ARDMK_DIR = /usr/share/arduino/Arduino-Makefile
BOARD_TAG     = uno
MONITOR_PORT  = /dev/ttyACM*
ARDUINO_LIBS = Servo
BOARDS_TXT = $(ARDUINO_DIR)/hardware/arduino/avr/boards.txt
include $(ARDMK_DIR)/Arduino.mk
```
- Should look at the [variables](https://github.com/sudar/Arduino-Makefile/blob/master/arduino-mk-vars.md) you can define to setup the makefile