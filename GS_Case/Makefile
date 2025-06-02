## Make driven command interface to simplifiy Docker use

.PHONY: docker-build docker-run 

## Build the docker image
docker-build: 
	docker build -t lora/fastapi .

## Mount host file system and run the docker image
docker-run:
	docker run --rm -v "$(PWD):/code" --name CRS_lora_fastapi -p 80:80 lora/fastapi
