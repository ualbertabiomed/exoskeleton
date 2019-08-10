# Exoskeleton Project
To design an upper body exoskeleton which would reduce the chance of repetitive strain injury and provide feedback on the users's posture. 

## Getting started
These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

## Installation
Based on your operating system, follow the guides below to install the Arduino IDE and pySerial followed by cloning this repo to your local system. After this, the Makefile must be edited for each user.

### Windows 
#### Install Cygwin
- Cygwin is an open source project which provides the functionality of Linux distribution on Windows
1. Download Cygwin 32 or 64-bit version (according to your requirement) from the following page and save it to a known location (ex. the Desktop),
   - https://www.cygwin.com/
2. When a waning window pops up, click Yes.
3. Click Next.
4. Choose 'Install from Internet' option and then click Next.
5. Choose your desired directory by clicking on Browse button. Choose 'All Users (Recommended)' and then click Next.
6. Choose the local package directory where you would like your installation files to be placed (dont delete this folder in the future; its name will be long and weird, mine is http%3a%2f%2fcygwin.mirror.constant.com%2f). Click Next.
7. Choose 'Direct Connection' if you are using Internet with plug and play device. Click Next.
8. Choose any download site preferably "cygwin.mirror.constant.com" and then click Next.
9. In the top left corner of the package manager click the drop down list named view and select the option ‘Category’
10. To install a package you will first find it using the search bar and navigating to the correct category of software. Beside each package there is a column titled ‘new’. In this box if you see the label 'skip' in front of a package then this package is not currently installed. To select a package for installation click the drop down and select the highest version number which is a non-beta release (the latest version will typically be lower in the list). If you see the label 'Keep', leave as it is - this means that there is a current version of this package already installed. In the following instructions you will see many packages returned by your search results. It is very important that you select the packages whose names exactly match what is listed below. Install the latest (non-beta versions) of the following minimal packages for Cygwin:

    - gcc-core: GNU Compiler Collection (C,OpenMP) - To select this package, type 'gcc-core' in search bar and hit Enter. Expand 'Devel'.

    - make: The GNU version of the 'make' utility - Again type 'make' in the search bar and hit Enter. Again expand 'Devel' by clicking corresponding + button.

    - vim: Vi Improved - enhanced vi editor - Do the same but this time search vim and it should be in 'Editors' category.
  
    - Perl: Perl programming language interpreter - Search perl and expand the interpreters category. 

    - Python2: Python programming language interpreter - Search python and expand the interpreters category.

11. Click Next.
12. Review the changes and then click Next.
13. Wait for the download and installation to finish
14. Keep default check boxes of placing an icon on the Desktop (this is important because you use this shortcut to open the cygwin terminal) and then click Finish.
15. Move the cygwin setup application that you downloaded in step 1 to the cygwin installation folder from step 5. If you ever need to install a new package, update a package, or remove a package, you simply rerun this ‘setup’ tool (it doesnt actually reinstall anything at this point it just updates files).

#### Install pySerial
1. Open the cygwin terminal using the shortcut that was placed on your desktop during installation.
2. Enter the following command to install pip (a python package installer) by copying and pasting:
   - ```/usr/bin/python2 -m ensurepip```
3. Now enter the following to use pip to install the pySerial python package:
   - ```pip install pyserial```

#### Install PuTTY
1. Follow this guide to install:
   - https://www.ssh.com/ssh/putty/windows/install
2. Open PuTTY. Create a new saved session for the com4 serial port:
3. In the textbox under ‘Saved Sessions’ type a name to save the settings we are about to set under (example scom4).
4. Click the ‘Serial’ radio dial above.
5. In the ‘Serial line’ textbox, which should have just appeared, type ‘COM4’ and set speed to 9600.
6. On the left hand side in the categories pane, under Session, click logging.
7. Click the radio dials “All session output” and “Always overwrite it”.
8. Optionally click browse and choose where you want serial output logs to be saved and under what name (keeping the default installation to be in your PuTTY installation directory with the name putty.log is fine).
9. On the left hand side in the categories pane, click Session.
10. Click the save button on the right hand side.

#### Clone exoskeleton repo
Clone the exoskeleton repository to your computer (see other section…)

#### Possible issues 
- Anti-virus blocks cygwin bash commands (mkdir, rm, cd , etc). Permission denied errors can be fixed if the user disables anti-virus for cygwin.
- If you get an error message when performing a `make upload` command (saying something about being unable to configure the port) then try opening the Arduino IDE and then compile and upload a blank sketch (with just an empty setup and loop) before you try ‘make upload’ again. Something about uploading a sketch using the Arduino IDE seems to help windows find the port in the future...

### MacOS
**Pre - requisites:** basic knowledge of terminal and navigating macOS file system

**1. Install Arduino IDE**
- [Install](https://www.arduino.cc/en/main/software)
      - Choose MacOS X
- Installation Guide for MacOS X [here](https://www.arduino.cc/en/guide/macOSX) 

**2. Install MacPorts**
- [Install](https://guide.macports.org/chunked/installing.macports.html)
      - Download the correct package installer based on MacOS version 
      - To check MacOS version, click on the "Apple" on the top left and then "About this Mac"

**3. Install pySerial**
- In the terminal:
```
sudo port install py27-serial
```

**3. Clone exoskeleton repo**
```
git clone https://github.com/ualbertabiomed/exoskeleton.git
```

**4. Configure the Makefile**
- Uncomment only the MacOS section

### Linux 
**Pre-requisites:** 
- Basic knowledge of bash terminal and navigating Linux file system
- Sudo privileges


<!-- **1. Install Arduino IDE**
- [Install](https://www.arduino.cc/en/main/software)
      - Choose Linux (32bit or 64bit depending on your computer) 
- Installation Guide for Linux [here](https://www.arduino.cc/en/Guide/Linux) 
      - Follow up to and including "Run the install script"
- Store the installation folder somewhere you will remember (ex. `home` directory or `/usr/share`)
- *Troubleshooting* you may need to run `chmod +x install.sh` prior to running `./install.sh` 

**2. Install pySerial**
- In the terminal:
```
pip install pyserial
```
- Note: pip should already be installed on most linux machines if python 2 or python 3 is installed 

**3. Install Arduino Makefile**
```
sudo apt-get install arduino-mk
echo 'export ARDUINO_DIR={Path to Arduino installation}' >> ~/.bashrc
echo 'export ARDMK_DIR=/usr/share/arduino' >> ~/.bashrc
echo 'export AVR_TOOLS_DIR=/usr/include' >> ~/.bashrc
echo 'export BOARD_TAG=mega2560' >> ~/.bashrc
```
- Uncomment only the Linux section -->

**1. Clone exoskeleton repo**
```
git clone https://github.com/ualbertabiomed/exoskeleton.git
cd exoskeleton
```

**2. Run install script**
```
sudo chmod +x setup/install-linux.sh
sudo ./setup/install-linux.sh
```

## Troubleshooting
`-mmcu=` error. *Solution:* define BOARD_TAG and BOARD_SUB 

---
## Running the tests
Explain how to run the automated tests for this system.
Break down into end to end tests: Explain what thest tests test and why.
       .
## License 
This project is licensed under ...        .
 
## Acknowledgements
Hat tip to anyone whose code was used, inspiration, references, etc.
