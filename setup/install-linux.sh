#!/bin/sh

# This script will install the exoskeleton project on Linux
# Tested on Ubuntu 16.04
# Note: might need to change permissions if cannot run script with
# sudo chmod +x install-linux.sh

# Declare variables here
# Default mode is to install.
UNINSTALL=false
RESOURCE_NAME=arduino-arduinoide

# Get absolute path from which this script file was executed
# (Could be changed to "pwd -P" to resolve symlinks to their target)
SCRIPT_PATH=$( cd $(dirname $0) ; pwd )
cd "${SCRIPT_PATH}"
printf "SCRIPT PATH=${SCRIPT_PATH}\n"


############################################################################################################
# COPIED FROM README
# **1. Install Arduino IDE**
# - [Install](https://www.arduino.cc/en/main/software)
#       - Choose Linux (32bit or 64bit depending on your computer) 
# - Installation Guide for Linux [here](https://www.arduino.cc/en/Guide/Linux) 
#       - Follow up to and including "Run the install script"
# - Store the installation folder somewhere you will remember (ex. `home` directory or `/usr/share`)
# - *Troubleshooting* you may need to run `chmod +x install.sh` prior to running `./install.sh` 

# **2. Install pySerial**
# - In the terminal:
# ```
# pip install pyserial
# ```
# - Note: pip should already be installed on most linux machines if python 2 or python 3 is installed 

# **3. Install Arduino Makefile**
# ```
# sudo apt-get install arduino-mk
# echo 'export ARDUINO_DIR={Path to Arduino installation}' >> ~/.bashrc
# echo 'export ARDMK_DIR=/usr/share/arduino' >> ~/.bashrc
# echo 'export AVR_TOOLS_DIR=/usr/include' >> ~/.bashrc
# echo 'export BOARD_TAG=mega2560' >> ~/.bashrc
# ```
# - Uncomment only the Linux section
############################################################################################################
# Install by simply copying desktop file (fallback)
simple_install_f() {

  # Create a temp dir accessible by all users
  TMP_DIR=`mktemp --directory`

  # Create *.desktop file using the existing template file
  sed -e "s,<BINARY_LOCATION>,${SCRIPT_PATH}/arduino,g" \
      -e "s,<ICON_NAME>,${SCRIPT_PATH}/lib/arduino.png,g" "${SCRIPT_PATH}/lib/desktop.template" > "${TMP_DIR}/${RESOURCE_NAME}.desktop"

  mkdir -p "${HOME}/.local/share/applications"
  cp "${TMP_DIR}/${RESOURCE_NAME}.desktop" "${HOME}/.local/share/applications/"

  mkdir -p "${HOME}/.local/share/metainfo"
  cp "${SCRIPT_PATH}/lib/appdata.xml" "${HOME}/.local/share/metainfo/${RESOURCE_NAME}.appdata.xml"

  # Copy desktop icon if desktop dir exists (was found)
  if [ -d "${XDG_DESKTOP_DIR}" ]; then
   cp "${TMP_DIR}/${RESOURCE_NAME}.desktop" "${XDG_DESKTOP_DIR}/"
   # Altering file permissions to avoid "Untrusted Application Launcher" error on Ubuntu
   chmod u+x "${XDG_DESKTOP_DIR}/${RESOURCE_NAME}.desktop"
  fi

  # Add symlink for arduino so it's in users path
  echo "" # Ensure password request message is on new line
  if ! ln -s ${SCRIPT_PATH}/arduino /usr/local/bin/arduino; then
      echo "Adding symlink failed. Hope that's OK. If not then rerun as root with sudo."
  fi

  # Clean up temp dir
  rm "${TMP_DIR}/${RESOURCE_NAME}.desktop"
  rmdir "${TMP_DIR}"

#   # Launching arduino-linux-setup.sh script
#   #./arduino-linux-setup.sh $(whoami)

}

# Uninstall by simply removing desktop files (fallback), incl. old one
simple_uninstall() {

  # checks if file exists are removes if it does
  if [ -f "${HOME}/.local/share/applications/arduino.desktop" ]; then
    rm "${HOME}/.local/share/applications/arduino.desktop"
  fi

  # Remove symlink for arduino
  echo "" # Ensure password request message is on new line
  if ! rm /usr/local/bin/arduino; then
      echo "Removing symlink failed. Hope that's OK. If not then rerun as root with sudo."
  fi

}

# Shows a description of the available options
display_help_f() {
  printf "\nThis script will install the necessary dependencies for the exoskeleton project\n"
  # if ! xdg_exists_f; then
  #   printf "\nxdg-utils are recommended to be installed, so this script can use them.\n"
  # fi
  printf "\nOptional arguments are:\n\n"
  printf "\t-u, --uninstall\t\tRemoves environmental variables.\n\n"
  printf "\t-h, --help\t\tShows this help again.\n\n"
}

# Check for provided arguments
while [ $# -gt 0 ] ; do
  ARG="${1}"
  case $ARG in
      -u|--uninstall)
        UNINSTALL=true
        shift
      ;;
      -h|--help)
        display_help_f
        exit 0
      ;;
      *)
        printf "\nInvalid option -- '${ARG}'\n"
        display_help_f
        exit 1
      ;;
  esac
done

if [ ${UNINSTALL} = true ]; then
  printf "Uninstalling exoskeleton project..."
  simple_uninstall_f
else
  printf "Installing exoskeleton project..."
  simple_uninstall_f
  simple_install_f
fi
printf " done!\n"

exit 0
