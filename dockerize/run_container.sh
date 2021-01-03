cd ../sim/
export SIM_PATH=$(pwd)
cd ../dockerize/
docker run \
    -a stdin \
    -a stdout \
    -a stderr \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $SIM_PATH:/usr/src \
    -w /usr/src \
    --name "exosim" \
    exosim_image \
    python3 sim.py
echo "exosim_image successfully built and exosim container successfully run"