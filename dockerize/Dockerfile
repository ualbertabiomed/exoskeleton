FROM ubuntu:18.04
LABEL version="0.1"
LABEL description="dockerfile for running pybullet simulations"

# Upgrade installed packages
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get upgrade -y && apt-get clean

# Install dependencies
RUN apt-get update \
    && apt-get install -y libxext-dev libxrender-dev libxtst-dev \
    && apt-get clean


# Install pip and pybullet
RUN apt-get update \
  && apt-get install -y python3-pip python3-dev \
  && cd /usr/local/bin \
  && ln -s /usr/bin/python3 python \
  && pip3 install --upgrade pip

# Install dependencies
COPY requirements.txt .
RUN pip3 install -r requirements.txt

# Create directory
RUN cd /usr/
RUN mkdir src