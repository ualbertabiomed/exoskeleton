#!/bin/bash

###### Get script location
# https://stackoverflow.com/a/246128
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null && pwd )"

###### Update/ install git submodules
#git submodule update --init --recursive


###### Setup ros path
cd ~
mkdir -p ~/ros/src
cd ~/ros
catkin_make

echo "source ~/ros/devel/setup.bash" >> ~/.bashrc
source ~/ros/devel/setup.bash

## Link packages
cd ~/ros/src
ln -s /exoskeleton
# later if we have more packages, we can just add a link here

cd ~/ros
# update rosdeps first
rosdep install --from-paths src --ignore-src -r -y
catkin_make





