all: build_image run_container

build_image:
	docker build -t exosim_image .

run_container:
	chmod +x run_container.sh
	./run_container.sh

enter:
	docker exec -it exosim /bin/bash

show:
	docker images
	docker ps -a

clean:
	docker stop exosim
	docker rm exosim
	docker rmi exosim_image
