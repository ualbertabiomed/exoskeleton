# exoskeleton
To design an upper body exoskeleton which would reduce the chance of repetitive strain injury and provide feedback on the users's posture. 

Getting started: 
These instructions will get you a cope of the project up and running on your local machine for development and testing purposes.

Prerequisties: 
python 3

---
## Installing: 
A step by step series of examples that tell you how to get a development env running. Say what the step will be, and repeat; end with an example of getting some data out of the system or using it for a little demo.
- As of July 6, 2019 latest Arduino IDE version: 1.8.9

Based on your operating system, follow the guides below to install the Arduino IDE and pyserial followed by cloning this repo to your local system. After this, the Makefile must be edited for each user.

### Windows 

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

**3. Install Pyserial**
- In the terminal:
```
sudo port install py27-serial
```

**3. Clone exoskeleton repo**
```
git clone https://github.com/ualbertabiomed/exoskeleton.git
```

**4. Configure the Makefile**


### Linux 
**Pre - requisites:** basic knowledge of terminal and navigating linux file system

**1. Install Arduino IDE**
- [Install](https://www.arduino.cc/en/main/software)
      - Choose Linux (32bit or 64bit depending on your computer) 
- Installation Guide for Linux [here](https://www.arduino.cc/en/Guide/Linux) 
      - Follow up to and including "Run the install script"
- Store the installation folder somewhere you will remember (ex. `home` directory or `/usr/share`)
- *Troubleshooting* you may need to run `chmod +x install.sh` prior to running `./installed.sh` 

**2. Install Pyserial**
- In the terminal:
```
pip install pyserial
```
- pip should already be installed on most linux machines if python 2 or python 3 is installed 

**3. Clone exoskeleton repo**
```
git clone https://github.com/ualbertabiomed/exoskeleton.git
```

**4. Configure the Makefile**


### Troubleshooting

---
Running the tests: 
Explain how to run the automated tests for this system.
Break down into end to end tests: Explain what thest tests test and why.

Coding style tests:
Explain what these tests test and why.

Development: 
Add additional notes about how to deploy this on a live system.

Built with: 

Contributing: 
Please read       for details on our code of conduct and the process for submitting pull requests to us.

Versioning: 
We use       for versioning. For the version available, see.        .
Authors: 
      - Initial work -
See also the list of contributors who participated in this project.

License: 
This project is licensed under the         .
 
 Acknowledgements: 
 Hat tip to anyone whose code was used
 Inspiration
 etc
