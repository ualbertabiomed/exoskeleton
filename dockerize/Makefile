default: create run

create:
	docker build -t exosim_image .

run:
	chmod +x run_container.sh
	./run_container.sh
	echo "removing exosim container..."
	docker rm exosim

show:
	docker images
	docker ps -a

clean:
	docker stop exosim
	docker rm exosim
	docker rmi exosim_image
