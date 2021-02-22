default: build start enter

build:
	docker build -t rosnoetic_img .
	echo "rosnoetic_img successfully built"

start:
	docker run \
    	-d \
    	-i \
    	--env="DISPLAY" \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
    	--name "rosnoetic_cntr" \
    	rosnoetic_img
	echo "rosnoetic_cntr successfully started up"

enter:
	echo "entering rosnoetic_cntr..."
	xhost local:root
	docker exec -it rosnoetic_cntr /bin/bash

show:
	docker images
	docker ps -a

clean:
	docker stop rosnoetic_cntr
	docker rm rosnoetic_cntr
	docker rmi rosnoetic_img
