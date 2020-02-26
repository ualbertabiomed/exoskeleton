FROM ros:melodic-robot

RUN apt-get update && apt-get install -y tmux \
                                         vim \
                                         nano \
                                         python-pip

RUN python2 -m pip install monotonic \
    scipy \
    numpy \
    Sphinx \
    numpydoc \
    nose \
    pykalman \
    monotonic \
    odrive


SHELL ["/ros_entrypoint.sh", "bash", "-c"]


COPY . /exoskeleton

RUN ( cd exoskeleton && ./unpack.sh )
