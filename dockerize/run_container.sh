cd ../exo/
export EXO_PATH=$(pwd)
cd ../dockerize/
docker run \
    -d \
    --env DISPLAY=$DISPLAY \
    --volume $EXO_PATH:/usr/src/exo \
    --name "exosim" \
    exosim_image
echo "exosim_image successfully built and exosim container successfully run"