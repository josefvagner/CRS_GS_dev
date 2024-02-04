## Make driven command interface to simplifiy RPI Pico development in Docker

.PHONY: docker-build docker-run 

## Build the docker image
docker-build: 
	docker build -t rpi/pico .

## Mount host file system and run the docker image
docker-run: 
	docker run --rm -it -v "$(PWD):/home/dev" --name sx1280_dev rpi/pico:latest
