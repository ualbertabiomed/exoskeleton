cd ../exo/
export EXO_PATH=($pwd)
cd ../dockerize/
docker run -i \
    --env DISPLAY=$DISPLAY \
    --volume $EXO_PATH:/usr/src/app \
    --name "exosim" \
    exosim_image
echo "exosim_image successfully built and exosim container successfully run"