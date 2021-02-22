## Running ROS-Gazebo Docker
1. Navigate to the exoskeleton/dockerize directory
    - `cd PATH_TO_DIR`
2. Build the image and run the simulation using:
    - `make`

## Additional Commands
To build the rosnoetic_img image:

`make build`

To run and start up the rosnoetic_cntr container:

`make start`

To enter the rosnoetic_cntr container (rosnoetic_cntr must have a status of Up for this command to work - can check status of all containers using `docker ps -a`):

`make enter`

To show all current images and containers:

`make show`

To stop and remove the rosnoetic_cntr container and remove the rosnoetic_img:

`make clean`

## Basic Docker Commands
To list docker images you have installed run:

`docker images`

To list all docker containers run:

`docker ps -a`

## Verify Successful Install
Follow the steps below to verify that the dockerfile has successfully been installed and is capable of opening a GUI:

1. Run `roscore &`
2. Run `rosrun turtlesim turtlesim_node` and a window should open displaying a turtle against a blue screen

See [here](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes) for more info
