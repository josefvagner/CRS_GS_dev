BIN_SUFFIX := .uf2

## Make driven command interface to simplify RPI Pico development in Docker
.PHONY: docker-build docker-run

## Build the project ----------------------------------------------------------
all:
	mkdir -p build
	cmake -S ./ -B ./build/

build: all
	cmake --build ./build

demo: all
	cmake --build ./build/demo

# 'make panels TARGET=Power_board' or 'make panels'
panels: all
ifeq ($(TARGET),)
	cmake --build ./build/panels
	mv ./build/panels/**/*$(BIN_SUFFIX) ./panels/
else
	cmake --build ./build/panels/$(TARGET)
	cp -f ./build/panels/$(TARGET)/*$(BIN_SUFFIX) ./panels/$(TARGET)$(BIN_SUFFIX)
endif

libs: all
	cmake --build ./build/libs

source: all
	cmake --build ./build/

clean:
	rm -rf panels/*.elf
	rm -rf build

## Build the docker image -----------------------------------------------------
docker-build:
	docker build -t rpi/pico .

## Mount host file system and run the docker image
docker-run:
	docker run --rm -it -v "$(PWD):/home/dev" --name cansatDev rpi/pico:latest
