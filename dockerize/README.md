## Running ROS-Gazebo Docker
1. Navigate to the exoskeleton/dockerize directory
    - `cd PATH_TO_DIR`
2. Build the image and run the simulation using:
    - `make`

## Additional Commands
To show all current images and containers:

`make show`

To stop and remove the exosim container and remove the exosim_image:

`make clean`

## Basic Docker Commands
To list docker images you have installed run:

`docker images`

To list all docker containers run:

`docker ps -a`