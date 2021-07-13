# Exoskeleton - Controller (Raspberry Pi 4) Code

## Summary

The controller directory is a catkin package, meant to be uploaded on the controller for the Exoskeleton.

## Upload Instructions

### Instructions for setting up a new Pi

#### Method 1: Docker (WIP)
1. Install Ubuntu 20.X
2. Create uab user with password “uabiomed”
3. Name device “uab-piX” where X is whatever number Pi this is
4. Do the following in order:
   1. “sudo apt update”
   2. “sudo apt upgrade”
   3. “sudo apt install curl git make” # Get packages to instal docker and our repo
   4. “curl -sSL https://get.docker.com | sh” # Install docker from website, will take a bit
   5. “sudo usermod -aG docker uab” # Adds uab to docker group
   6. Logout and log back in
   7. “docker run hello-world” # Verify everything worked
5. Setup a github SSH key by following the steps listed here, specifically:
   1. Generating a new SSH key and adding it to the agent
   2. Adding a new SSH key to your github account
   3. Testing your SSH connection

#### Method 2: Native
1. Install Ubuntu 20.X
2. Create uab user with password "uabiomed"
3. Name device "uab-piX" where X is whatever number Pi this is
4. Setup github SSH key (See step 5 above)
5. Pull exoskeleton repo
6. Install ROS as detailed [here](http://wiki.ros.org/noetic/Installation/Ubuntu)
7. Install motor dependencies, what these are is tbd...
8. Run launch file

## Style

Try to follow [this](http://wiki.ros.org/PyStyleGuide) style guide 